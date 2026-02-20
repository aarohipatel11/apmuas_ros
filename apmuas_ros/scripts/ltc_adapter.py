from controller_interface import ControllerInterface
from typing import List, Any, Dict
import numpy as np
from apmuas_ros.PID import PID, FirstOrderFilter
from apmuas_ros.drone_math import DroneMath
from drone_interfaces.msg import CtlTraj
import time
import rclpy
import math

# Controller Mode Enumerations
LTC_MODE = 0  
MPC_MODE = 1  

class LTCAdapter(ControllerInterface):
    """
    LTC (Level-Turn Controller) Adapter implementing ControllerInterface

    """
    def __init__(self):
        self.dz_filter : FirstOrderFilter = FirstOrderFilter(
            tau=0.5, dt=0.025, x0=0.0)
        self.yaw_filter : FirstOrderFilter = FirstOrderFilter(
            tau=0.3, dt=0.025, x0=0.0)
        
        self.dz_controller: PID = PID(
            kp=0.025, ki=0.0, kd=0.01,
            min_constraint=np.deg2rad(-12),
            max_constraint=np.deg2rad(10),
            use_derivative=True,
            dt = 0.025)
        
        self.roll_controller: PID = PID(
            kp=0.5, ki=0.0, kd=0.05,
            min_constraint=np.deg2rad(-40),
            max_constraint=np.deg2rad(40),
            use_derivative=True,
            dt = 0.025)
        
        self.cartesian_waypoints: List[List[float]] = []
        self.current_state: List[float] = [None, None, None, None, None, None, None]
        self.trajectory_command_history: List[Dict] = []
        self.trajectory_publisher = None
        self.controller_mode = LTC_MODE
        self.last_controller_mode = None
        # from main() in guidance_publisher.py
        self.aircraft_max_roll_deg: float = 40.0
        self.camera_range_m: float = 100.0
        self.num_loiters: int = 2
        self.radius_to_close: float = 15.0
        self.loiter_radius: float = DroneMath.realtime_loiter_radius(
            mount_angle_phi_deg=self.aircraft_max_roll_deg,
            cam_range_m=self.camera_range_m,
            roll_limit_deg=self.aircraft_max_roll_deg)
        self.current_target_index: int = 0


    def get_commands(self, current_state: List[float], target_state: List[float]) -> CtlTraj:
        """
        Calculate control commands to navigate from current state to target state.
        
        Args:
            current_state: [x, y, z, roll, pitch, yaw, airspeed] - where drone currently IS
            target_state: [x, y, z] - desired waypoint the drone WANTS TO REACH
            
        Returns:
            Control commands (CtlTraj)
        """
        if self.is_close(radius_to_close=self.radius_to_close, 
                     target_idx=self.current_target_index, 
                     loiter_radius=self.loiter_radius):
            # if close - LTC (loiter)
            aircraft_speed = current_state[6]
            loiter_time_sec = DroneMath.calculate_loiter_time(num_loiters=self.num_loiters,
                                                            loiter_radius=self.loiter_radius,
                                                            aircraft_velocity_mps=aircraft_speed)
            delta_time = 0
            current_time = time.time()
            while delta_time < loiter_time_sec:
                delta_time = time.time() - current_time
                # is controller_state_machine necessary? calc_los instead?
                trajectory = self.controller_state_machine(target_index=self.current_target_index)
                rclpy.spin_once(self, timeout_sec=0.05)
            return trajectory
        else:
            # if far - bearing command
            return self.calculate_line_of_sight(self.current_target_index)
        
        # Input is Current State (x, y, z, phi (roll), theta (pitch), psi (yaw)), airspeed & Target State (x, y, z)
        # Psuedocode:
        # 1. Calculate distance: Using a^2 + b^2 = c^2 -> ((target x - current x)^2 + (target y - current y)^2) = c^2 -> then sqrt
        # 2. Check if close
        # 3. If close, Execute LTC (Level Turn Control), else execute bearing command.




    def calculate_line_of_sight(self, target_index:int) -> CtlTraj:
        """
        You need to calculate the trajectory based on the target position
        Remember the yaw command must be RELATIVE 
        """
        if self.current_state[0] is None:
            return
        
        dx:float = self.cartesian_waypoints[target_index][0] - self.current_state[0]
        dy:float = self.cartesian_waypoints[target_index][1] - self.current_state[1]
        dz:float = self.cartesian_waypoints[target_index][2] - self.current_state[2]
        # dz is already computed in the model so set setpoint as dz
        # and the current value as 0.0
        dz = self.dz_filter.filter(dz)
        dz = np.clip(dz, -10.0, 10.0)
        if self.dz_controller.prev_error is None:
            self.dz_controller.prev_error = 0.0
            
        pitch_cmd:float = self.dz_controller.compute(
            setpoint=dz,
            current_value=0.0,
            dt=0.05
        )
        pitch_cmd = np.clip(pitch_cmd, -np.deg2rad(12), np.deg2rad(10))
        
        dist: float = np.sqrt(dx**2 + dy**2)
        #print("Dist: ", dist)
        
        enu_yaw_rad:float = np.arctan2(dy, dx)
        #ned_yaw_cmd_rad:float = yaw_enu_to_ned(enu_yaw_rad)
        ned_yaw_rad = yaw_enu_to_ned(enu_yaw_rad)
        ned_yaw_state = yaw_enu_to_ned(self.current_state[5]) 
        rel_yaw_cmd:float = get_relative_ned_yaw_cmd(
            ned_yaw_state, ned_yaw_rad)
        rel_yaw_cmd = self.yaw_filter.filter(
            rel_yaw_cmd)
        # relative yaw command is already computed as error 
        # so we set setpoint to 0.0
        if self.roll_controller.prev_error is None:
            self.roll_controller.prev_error = 0.0
            

        #TODO: Keep it safe and say 35-45 -> Done at 40
        roll_cmd = self.roll_controller.compute(
            setpoint=rel_yaw_cmd,
            current_value=0.0,
            dt=0.05
        )
        # make sure the roll command has the same sign convention as 
        # the yaw command
        if rel_yaw_cmd < 0.0 and roll_cmd > 0.0:
            roll_cmd = -roll_cmd
        elif rel_yaw_cmd > 0.0 and roll_cmd < 0.0:
            roll_cmd = -roll_cmd
            
        roll_cmd = np.clip(roll_cmd, -np.deg2rad(40), np.deg2rad(40))     
        thrust_cmd:float = float(0.5)      
        # create a trajectory message
        trajectory: CtlTraj = CtlTraj()
        # trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.roll = [roll_cmd, roll_cmd]
        trajectory.pitch = [pitch_cmd, pitch_cmd]
        trajectory.yaw = [rel_yaw_cmd, rel_yaw_cmd]
        trajectory.thrust = [thrust_cmd, thrust_cmd]
        trajectory.idx = int(0)

        #TODO: return something to record all these commands -> Done
        self.trajectory_publisher.publish(trajectory)

        trajectory_dict: Dict[str, float] = {
            'roll': roll_cmd,
            'pitch': pitch_cmd,
            'yaw': rel_yaw_cmd,
            'thrust': thrust_cmd
        }

        self.trajectory_command_history.append(trajectory_dict)
        return trajectory
    
    # From drone_math.py
    def calculate_loiter_time(self, num_loiters:int, loiter_radius: float, aircraft_velocity_mps: float) -> float: 
        loiter_circumference: float = (2) * (np.pi) * (loiter_radius)
        total_distance: float = loiter_circumference * num_loiters
        loiter_time = (total_distance) / aircraft_velocity_mps
        return loiter_time
    
    # Helper Methods/Dependencies for Calculate LOS
    
    def yaw_enu_to_ned(self, enu_yaw: float) -> float:
        """
        Convert yaw from ENU to NED.
        
        The conversion is symmetric:
        yaw_ned = (pi/2 - yaw_enu) wrapped to [-pi, pi]

        Parameters:
        yaw_enu (float): Yaw angle in radians in the ENU frame.
        
        Returns:
        float: Yaw angle in radians in the NED frame.
        """
        yaw_ned = np.pi/2 - yaw_enu
        return wrap_to_pi(yaw_ned)

    def get_relative_ned_yaw_cmd(
        current_ned_yaw:float, 
        inert_ned_yaw_cmd:float) -> float:

        yaw_cmd:float = inert_ned_yaw_cmd - current_ned_yaw
        
        # wrap the angle to [-pi, pi]
        return wrap_to_pi(yaw_cmd)
    
    def wrap_to_pi(angle:float) -> float:
        """
        Wrap an angle in radians to the range [-pi, pi].

        Parameters:
            angle (float): Angle in radians.
        
        Returns:
            float: Angle wrapped to [-pi, pi].
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    
    # State Management 
    
    def controller_state_machine(self, target_index: int) -> None:
        """
        Executes the appropriate controller logic based on the current control mode.
         
        Args:
            target_index (int): Index of the current target waypoint.

        Returns:
            None     
        """
        if self.controller_mode == LTC_MODE:
            if self.last_controller_mode != LTC_MODE:
                print("Using LTC controller")
            self.calculate_line_of_sight(target_index)

        elif self.controller_mode == MPC_MODE:
            if self.last_controller_mode != MPC_MODE:
                print("Using MPC controller")
            # Add MPC logic here
            
        else:
            print(f"Unknown controller_mode: {self.controller_mode}, defaulting to LTC.")
            self.calculate_line_of_sight(target_index)


        # Update the previous controller mode for tracking
        self.last_controller_mode = self.controller_mode



    #TODO: define this function by calculating current to target location -> Done
    #TODO: factor in the buffer here
    def is_close(self, radius_to_close: float, target_idx:int, loiter_radius:float) -> bool:
        #have two checks here, one to make sure that the altitude is acceptable enough for the camera range 
        # The other this to make sure that the loiter radius is met 
        radius_xy_good: bool = False
        altitude_z_good: bool = False
        #TODO: Checks here
        # offset_x: float = np.sin(self.current_state[5]) * loiter_radius
        # offset_y: float = np.cos(self.current_state[5]) * loiter_radius
        x:float = ((self.cartesian_waypoints[target_idx][0] - self.current_state[0]) + 0) **2
        y: float = ((self.cartesian_waypoints[target_idx][1] - self.current_state[1]) + 0)**2
        distance_from_target: float =  math.sqrt(x + y)
        if distance_from_target <= radius_to_close:
            radius_xy_good = True
        if abs(self.cartesian_waypoints[target_idx][2] - self.current_state[2]) <= 5: 
            altitude_z_good = True

        if radius_xy_good and altitude_z_good:
            return True
        return False

    
    # Need imports
    # FirstOrderFilter (PID.py, class), -> DONE
    # PID (PID.py, class), -> DONE
    # CtlTraj (_ctl_traj.py, class) -> DONE

    # New Functions
    def check_if_loiter_done() -> Any:
        pass