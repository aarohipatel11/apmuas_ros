#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class SayHiService(Node):
    def __init__(self):
        super().__init__('say_hi_service')
        self.srv = self.create_service(Trigger, 'say_hi', self.say_hi_callback)
        self.get_logger().info('Say Hi Service is ready!')
        self.call_count = 0

    def say_hi_callback(self, request, response):
        self.call_count += 1
        response.success = True
        response.message = f"Hi! #{self.call_count}"
        self.get_logger().info(f'Said hi #{self.call_count} times')
        return response

def main(args=None):
    rclpy.init(args=args)
    say_hi_service = SayHiService()
    rclpy.spin(say_hi_service)
    say_hi_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()