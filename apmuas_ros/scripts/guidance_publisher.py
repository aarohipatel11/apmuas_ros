#!/usr/bin/env python3


#The one that actually has changes on the todos
import rclpy
import math
import numpy as np
import mavros
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from nav_msgs.msg import Odometry
from drone_interfaces.msg import Telem, CtlTraj
#from apmuas_ros import CtlTraj
from apmuas_ros import rotation_utils as rot_utils
from re import S
from typing import List, Dict, Any
from mavros.base import SENSOR_QOS
from apmuas_ros.PID import PID, FirstOrderFilter
from apmuas_ros.drone_math import DroneMath, geodetic_to_cartesian, convert_all_to_cartesian
from apmuas_ros.drone import DroneCommander
import time
"""
For this application we will be sending roll, pitch yaw commands to the drone
"""


def yaw_enu_to_ned(yaw_enu:float)-> float:
    """
    Convert yaw angle from ENU to NED.
    
    The conversion is symmetric:
        yaw_ned = (pi/2 - yaw_enu) wrapped to [-pi, pi]

    Parameters:
        yaw_enu (float): Yaw angle in radians in the ENU frame.
        
    Returns:
        float: Yaw angle in radians in the NED frame.
    """
    yaw_ned = np.pi/2 - yaw_enu
    return wrap_to_pi(yaw_ned)

def wrap_to_pi(angle:float) -> float:
    """
    Wrap an angle in radians to the range [-pi, pi].

    Parameters:
        angle (float): Angle in radians.
    
    Returns:
        float: Angle wrapped to [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def get_relative_ned_yaw_cmd(
        current_ned_yaw:float, 
        inert_ned_yaw_cmd:float) -> float:

    yaw_cmd:float = inert_ned_yaw_cmd - current_ned_yaw
        
        # wrap the angle to [-pi, pi]
    return wrap_to_pi(yaw_cmd)

class GuidancePublisher(Node):
    """
    GOAL want to publish a roll,pitch,yaw trajectory to the drone to get
    to the target location
    HINTS Not in order
    - Remember the current coordinate system is in ENU need to convert to NED
    - Might need to add some safety checks
    - Need to calculate something to get the roll, pitch, yaw commands
        - Yaw and roll control the lateral motion
        - Pitch control the vertical motion  
    - Need to subscribe to something else besides the mavros state
    """

    def __init__(self, ns=''):
        super().__init__('pub_example')

        self.trajectory_publisher: Publisher = self.create_publisher(
            CtlTraj, 'trajectory', 10)

        self.state_sub: Subscription = self.create_subscription(
            mavros.local_position.Odometry,
            'mavros/local_position/odom',
            self.mavros_state_callback,
            qos_profile=SENSOR_QOS)
        
        self.drone_commander: DroneCommander = DroneCommander(
            master_link= 'udpin:127.0.0.1:14553'
        )

# TODO: A method to get the home lat and lon position - instead of hardcoding it - double check that this is the home position [0]
        
        self.mission_items: List[Dict[str, Any]] = self.drone_commander.read_mission_items()
        print("Waiting for mission items...")
        self.does_mission_items_exist()
        print("Mission items exist, proceeding with waypoints...")

        self.home_lat: float = self.mission_items[0]['x']  # Assuming 'x' is latitude
        self.home_lon: float = self.mission_items[0]['y']  # Assuming 'y' is longitude


        self.cartesian_waypoints: List[List[float]] = [
            [-145, 2, 60],
            [150, 227, 70]
        ]

        # TODO need to keep listening in case we have changed waypoints
        self.cartesian_waypoints: List[List[float]] = self.convert_waypoints_to_cartesian()

        self.current_target_index: int = 0


        # smoothing for controller
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
        
        
        self.current_state: List[float] = [
            None,  # x
            None,  # y
            None,  # z
            None,  # phi
            None,  # theta
            None,  # psi
            None   # airspeed
        ]

        self.trajectory_command_history :List[Dict[str, float]] = [

        ]
    def does_mission_items_exist(self) -> None:
        """
        Checks if mission items exist.

        Args:
            None
        
        Returns:
            None
                
        In this method we have to check if its less than 1 because mission planner caches the first waypoint i.e.:
        [
            [home_lat, home_lon, 0.0] is always in this list no matter what
        ]
        
        It is less than or equal to 1 because the first item is a repeat/ghost waypoint that Mission Planner adds
        Starting from 1 accounts for this repeat
        """

        while len(self.mission_items) <= 1: 
            self.mission_items: List[Dict[str, Any]] = self.drone_commander.read_mission_items()
    
    def convert_waypoints_to_cartesian(self) -> List[List[float]]:
        """
        Converts geodetic coordinates (latitude, longitude) of waypoints to Cartesian coordinates.

        Args:
            None

        Returns:
            List[List[float]]: A list of Cartesian coordinates (x, y) for each waypoints, excluding the home location.
        
        """

        print(self.mission_items)
        print(self.home_lat, self.home_lon)

        origin_lat: float = self.home_lat
        origin_lon: float = self.home_lon
        
        #Empty list to hold the coordinates
        coord_list = []
        coord_list.append([origin_lat, origin_lon, 0.0])  # Add origin point

        # Loop through the mission items and extract the x and y coordinates
        for i, item in enumerate(self.mission_items):
            if i == 0:
                continue
            if 'x' in item and 'y' in item:
                coord_list.append([item['x'], item['y'], item['z']])
            else:
                print("Missing x or y in item:", item)
 

        print(coord_list)

        cartesian_coords = convert_all_to_cartesian(coord_list)

        print("Cartesian Coordinates:")
        for xy in cartesian_coords:
            print(f"X: {xy[0]:.2f}, Y: {xy[1]:.2f}")
        
        # Returns past 1 because the first coordinate is the home location
        return cartesian_coords[1:]

    def mavros_state_callback(self, msg: mavros.local_position.Odometry) -> None:
        """
        Converts NED to ENU and publishes the trajectory
        """
        self.current_state[0] = msg.pose.pose.position.x
        self.current_state[1] = msg.pose.pose.position.y
        self.current_state[2] = msg.pose.pose.position.z

        # quaternion attitudes
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll, pitch, yaw = rot_utils.euler_from_quaternion(
            qx, qy, qz, qw)

        self.current_state[3] = roll
        self.current_state[4] = pitch
        self.current_state[5] = yaw  # (yaw+ (2*np.pi) ) % (2*np.pi);

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        # get magnitude of velocity
        self.current_state[6] = np.sqrt(vx**2 + vy**2 + vz**2)
        
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
    
    def check_for_new_waypoints(self) -> None:
        """
        TODO: Check if the target waypoints have changed.
        
        This method should be called periodically to check if the target waypoints
        have been updated. If they have, it will reassign the cartesian_waypoints.
        """
        temp_mission_item = self.drone_commander.read_mission_items()
        
        if not temp_mission_item:
            print("No mission items found.")
            return
        
        temp_cartesian_waypoints : List[List[float]] = []
        
        print(temp_mission_item)

        origin_lat: float = temp_mission_item[0]['x']  # Assuming 'x' is latitude
        origin_lon: float = temp_mission_item[0]['y']  # Assuming 'y' is longitude

        #Empty list to hold the coordinates
        coord_list = []
        coord_list.append([origin_lat, origin_lon, 0.0])  # Add origin point

        # Loop through the mission items and extract the x and y coordinates
        for i, item in enumerate(temp_mission_item):
            if i == 0:
                continue
            if 'x' in item and 'y' in item:
                coord_list.append([item['x'], item['y'], item['z']])
            else:
                print("Missing x or y in item:", item)
 

        print(coord_list)

        temp_cartesian_waypoints = convert_all_to_cartesian(coord_list)

        print("New Cartesian Coordinates:")
        for xy in temp_cartesian_waypoints:
            print(f"X: {xy[0]:.2f}, Y: {xy[1]:.2f}")
        
        # Returns past 1 because the first coordinate is the home location
        #return cartesian_coords[1:]
        #GuidancePublisher.convert_waypoints_to_cartesian(temp_mission_item)
        # if self.cartesian_waypoints != temp_cartesian_waypoints[1:]:
        #     self.cartesian_waypoints = temp_cartesian_waypoints
        #     self.current_target_index = 0
        #     print("New waypoints detected, updating cartesian_waypoints.")

        if len(self.cartesian_waypoints) != len(temp_cartesian_waypoints[1:]):
            self.cartesian_waypoints = temp_cartesian_waypoints[1:]
            self.current_target_index = 0
            print("New waypoints detected, updating cartesian_waypoints.")
        else:
            difference: bool = False
            for i in range(len(self.cartesian_waypoints)):
                if self.cartesian_waypoints[i][0] != temp_cartesian_waypoints[i+1][0]:
                    difference = True
                    break
                if self.cartesian_waypoints[i][1] != temp_cartesian_waypoints[i+1][1]:
                    difference = True
                    break
        
            if difference:
                self.cartesian_waypoints = temp_cartesian_waypoints[1:]
                self.current_target_index = 0
                print("New waypoints detected, updating cartesian_waypoints.")


            
        

def main() -> None:
    rclpy.init()
    guidance_publisher:GuidancePublisher = GuidancePublisher()
    aircraft_max_roll_deg: float = 40.0
    alt_max_limit:float = 100.0 
    alt_min_limit:float = 40.0
    camera_range_m: float = 100.0
    number_of_loiters: int = 2
    cont_check_wait_time: float = 5.0

    ideal_aircraft_alt_m = DroneMath.compute_altitude_m(cam_range_m_alt = camera_range_m,
                                max_roll_tan = aircraft_max_roll_deg,
                                max_alt_m = alt_max_limit,
                                min_alt_m = alt_min_limit)
    
    #TODO: pass tghrough the ideal altitude when there is a change in target waypoints
    for target in guidance_publisher.cartesian_waypoints:
        target[2] = ideal_aircraft_alt_m

    ideal_loiter_radius = DroneMath.realtime_loiter_radius(mount_angle_phi_deg=aircraft_max_roll_deg,
                                                           cam_range_m=camera_range_m,
                                                           roll_limit_deg=aircraft_max_roll_deg)
    bubble_radius: float = 15.0

    print("Alt: ", ideal_aircraft_alt_m)
    print("Radius: ", ideal_loiter_radius)
    #TODO: Call drone_math to calc ideal radius and alt -> Done

    cont_checking_delta_time = 0.0
    cont_checking_current_time = time.time()
    cont_last_call_time = cont_checking_current_time



    while rclpy.ok():
        try:
            now = time.time()
            cont_checking_delta_time = now - cont_checking_current_time
            if (now - cont_last_call_time) >= cont_check_wait_time:
                guidance_publisher.check_for_new_waypoints()
                cont_last_call_time = now


            if guidance_publisher.current_state[0] is None:
                rclpy.spin_once(guidance_publisher, timeout_sec=0.05)
                continue

            '''
            send the roll and alt from calculated values above
            straight to flight controller
            call publish_traj function
            calculate how long it would take for x amount of loiters
            start_time 
            while loop once time is over 
            done, move on to next waypoint in target_waypoints
            else: 
                calculate_line_of_sight()
            '''
            if guidance_publisher.is_close(
                radius_to_close=bubble_radius, target_idx=guidance_publisher.current_target_index, loiter_radius=ideal_loiter_radius):
                aircraft_speed = guidance_publisher.current_state[6] 
                loiter_time_sec = DroneMath.calculate_loiter_time(num_loiters=number_of_loiters, 
                                                                  loiter_radius=ideal_loiter_radius,
                                                                  aircraft_velocity_mps=aircraft_speed)
            
                delta_time = 0
                current_time = time.time()

                while delta_time < loiter_time_sec:
                    delta_time = time.time() - current_time                    
                    current_trajectory = guidance_publisher.calculate_line_of_sight(
                        target_index=guidance_publisher.current_target_index)
                    # current_trajectory.roll = [aircraft_max_roll_deg, aircraft_max_roll_deg]
                    # guidance_publisher.trajectory_publisher.publish(current_trajectory)  
                    print(delta_time, loiter_time_sec)
                    rclpy.spin_once(guidance_publisher, timeout_sec=0.05)
                if guidance_publisher.current_target_index >= len(guidance_publisher.cartesian_waypoints) - 1:
                    print("I'm done, resetting to 0", guidance_publisher.current_target_index)
                    guidance_publisher.current_target_index = 0
                else:
                    print("incrementing counter", guidance_publisher.current_target_index)
            
                    guidance_publisher.current_target_index = (guidance_publisher.current_target_index + 1)
                
            else:
                guidance_publisher.calculate_line_of_sight(
                    target_index=guidance_publisher.current_target_index)
                # print("going to line of sight")
    
        
            rclpy.spin_once(guidance_publisher, timeout_sec=0.05)
        
        except KeyboardInterrupt:
            
            guidance_publisher.get_logger().info('Keyboard Interrupt')
            break
    
    guidance_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()