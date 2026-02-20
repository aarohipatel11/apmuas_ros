#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class GuidanceAlgorithmService(Node):
    def __init__(self):
        super().__init__('guidance_algorithm_service')
        self.srv = self.create_service(Trigger, 'set_guidance_algorithm', self.set_algorithm_callback)
        self.get_logger().info('Guidance Algorithm Service is ready!')
        self.current_algorithm = "None"

    def set_algorithm_callback(self, request, response):
        algorithm = getattr(self, 'selected_algorithm', 'Unknown')
        self.current_algorithm = algorithm
        
        self.get_logger().info(f' Algorithm changed to: {algorithm}')
        
        if algorithm == "LTC":
            self.get_logger().info('  → Loading Linear Time Control algorithm...')
        elif algorithm == "MPC":
            self.get_logger().info('  → Loading Model Predictive Control algorithm...')
        elif algorithm == "Other":
            self.get_logger().info('  → Loading Other guidance algorithm...')
        
        response.success = True
        response.message = f"Algorithm set to: {algorithm}"
        return response

def main(args=None):
    rclpy.init(args=args)
    service = GuidanceAlgorithmService()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()