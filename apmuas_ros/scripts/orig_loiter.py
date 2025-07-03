#!/usr/bin/env python3


#guidance_publisher or waypoint_manager with no changes
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
from apmuas_ros import rotation_utils as rot_utils
from re import S
from typing import List
from mavros.base import SENSOR_QOS
from apmuas_ros.PID import PID, FirstOrderFilter
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

        # subscribe to your target position
        self.target_sub: Subscription = self.create_subscription(
            Odometry,
            'target_position',
            self.calculate_trajectory,
            10)

        # smoothing for controller
        self.dz_filter : FirstOrderFilter = FirstOrderFilter(
            tau=0.5, dt=0.025, x0=0.0)
        self.yaw_filter : FirstOrderFilter = FirstOrderFilter(
            tau=0.4, dt=0.025, x0=0.0)
        
        self.dz_controller: PID = PID(
            kp=0.025, ki=0.0, kd=0.01,
            min_constraint=np.deg2rad(-12),
            max_constraint=np.deg2rad(10),
            use_derivative=True,
            dt = 0.025)
        
        self.roll_controller: PID = PID(
            kp=0.25, ki=0.0, kd=0.05,
            min_constraint=np.deg2rad(-40),
            max_constraint=np.deg2rad(40),
            use_derivative=True,
            dt = 0.025)
        
        self.target: List[float] = [
            None,  # x
            None,  # y
            None,  # z
        ]
        
        self.current_state: List[float] = [
            None,  # x
            None,  # y
            None,  # z
            None,  # phi
            None,  # theta
            None,  # psi
            None,  # airspeed
        ]

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
        
    def calculate_trajectory(self, target_msg:Odometry) -> CtlTraj:
        """
        You need to calculate the trajectory based on the target position
        Remember the yaw command must be RELATIVE 
        """
        if self.current_state[0] is None:
            return
        
        # stores target position in ENU in target class list
        self.target[0] = target_msg.pose.pose.position.x
        self.target[1] = target_msg.pose.pose.position.y
        self.target[2] = target_msg.pose.pose.position.z
        
        # calculate distance from current position to target position
        # dx, dy = lateral distance
        # dz = vertical distance
        dx:float = self.target[0] - self.current_state[0]
        dy:float = self.target[1] - self.current_state[1]
        dz:float = self.target[2] - self.current_state[2]
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
        print("dist", dist)
        
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
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.roll = [roll_cmd, roll_cmd]
        trajectory.pitch = [pitch_cmd, pitch_cmd]
        trajectory.yaw = [rel_yaw_cmd, rel_yaw_cmd]
        trajectory.thrust = [thrust_cmd, thrust_cmd]
        trajectory.idx = int(0)
        
        self.trajectory_publisher.publish(trajectory)
        

def main() -> None:
    rclpy.init()
    guidance_publisher:GuidancePublisher = GuidancePublisher()
    while rclpy.ok():
        try:
            if guidance_publisher.current_state[0] is None:
                rclpy.spin_once(guidance_publisher, timeout_sec=0.05)
                continue
            rclpy.spin_once(guidance_publisher, timeout_sec=0.05)
        
        except KeyboardInterrupt:
            
            guidance_publisher.get_logger().info('Keyboard Interrupt')
            break
    
    guidance_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()