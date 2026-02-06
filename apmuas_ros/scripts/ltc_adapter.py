from controller_interface import ControllerInterface
from typing import List, Any, Dict
import numpy as np
import math

class LTCAdapter(ControllerInterface):
    """
    LTC (Level-Turn Controller) Adapter implementing ControllerInterface

    """
    def __init__(self):
        pass
    
    #TODO: Add missing dependencies for calculate_los like state tracking, controllers/filters, and trajectory.publisher

    def get_commands(self, current_state: List[float], target_state: List[float]) -> Any:
        """
        Calculate control commands to navigate from current state to target state.
        
        Args:
            current_state: [x, y, z, roll, pitch, yaw, airspeed] - where drone currently IS
            target_state: [x, y, z] - desired waypoint the drone WANTS TO REACH
            
        Returns:
            Control commands (CtlTraj)
        """
        pass

    def calculate_line_of_sight(self, target_index:int) -> CtlTraj:
        """
        You need to calculate the trajectory based on the target position
        Remember the yaw command must be RELATIVE 
        """
        if self.current_state[0] is None:
            return
        

        #TODO: Index into target_waypoints properly with target_index parameter -> Done
        # stores target position in ENU in target class list
        # self.target_waypoints[target_index][0] = target_msg.pose.pose.position.x
        # self.target_waypoints[target_index][1] = target_msg.pose.pose.position.y
        # self.target_waypoints[target_index][2] = target_msg.pose.pose.position.z
        
        # calculate distance from current position to target position 
        # dx, dy = lateral distance
        # dz = vertical distance
        # offset_x: float = np.sin(self.current_state[5]) * loiter_radius
        # offset_y: float = np.cos(self.current_state[5]) * loiter_radius
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

    # State Management 
    
    # New Functions
    def check_if_loiter_done() -> Any:
        pass