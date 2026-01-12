#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger # Used for any simple action that doesn't need input data
from flask import Flask, jsonify
from flask_cors import CORS
import threading

class SayHiClient(Node):
    def __init__(self):
        super().__init__('say_hi_client')
        self.client = self.create_client(Trigger, 'say_hi')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for say_hi service...')
        self.get_logger().info('Connected to say_hi service!')

    def call_service(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        
        # Wait for the result without spinning the node
        timeout = 5.0
        start_time = self.get_clock().now()
        while not future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                return {'success': False, 'message': 'Service call timeout'}
        
        if future.result() is not None:
            response = future.result()
            return {'success': response.success, 'message': response.message}
        else:
            return {'success': False, 'message': 'Service call failed'}

app = Flask(__name__)
CORS(app)
ros_client = None

@app.route('/say_hi', methods=['GET', 'POST'])
def say_hi_endpoint():
    if ros_client is None:
        return jsonify({'error': 'ROS2 not ready'}), 500
    try:
        result = ros_client.call_service()
        return jsonify(result)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main(args=None):
    global ros_client
    rclpy.init(args=args)
    ros_client = SayHiClient()
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    print("Bridge is running on http://localhost:5000")
    
    # Use a timer to keep the node alive
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ros_client)
    executor.spin()
    
    ros_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()