#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from flask import Flask, jsonify, request
from flask_cors import CORS
import threading

class GuidanceAlgorithmClient(Node):
    def __init__(self):
        super().__init__('guidance_algorithm_client')
        self.client = self.create_client(Trigger, 'set_guidance_algorithm')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for guidance algorithm service...')
        self.get_logger().info('Connected to guidance algorithm service!')
        self.current_algorithm = "None"
        self.current_status = "Idle"

    def call_service(self, algorithm):
        # Mock: Check if we should accept the change
        # In real implementation, this could check state
        if self.current_status == "Tracking":
            return {
                'accepted': False,
                'status': 'Rejected',
                'algorithm': self.current_algorithm,
                'message': 'Cannot switch algorithm while tracking is active'
            }
        
        self.current_algorithm = algorithm
        request_msg = Trigger.Request()
        future = self.client.call_async(request_msg)
        
        timeout = 5.0
        start_time = self.get_clock().now()
        while not future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                return {
                    'accepted': False,
                    'status': 'Error',
                    'algorithm': self.current_algorithm,
                    'message': 'Service call timeout'
                }
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.current_status = "Active"
                self.get_logger().info(f' Algorithm changed to: {algorithm}')
                
                if algorithm == "LTC":
                    self.get_logger().info('  → Linear Time Control algorithm loaded')
                elif algorithm == "MPC":
                    self.get_logger().info('  → Model Predictive Control algorithm loaded')
                elif algorithm == "Other":
                    self.get_logger().info('  → Other guidance algorithm loaded')
                
                return {
                    'accepted': True,
                    'status': 'Active',
                    'algorithm': algorithm,
                    'message': f'{algorithm} algorithm activated'
                }
            else:
                return {
                    'accepted': False,
                    'status': 'Rejected',
                    'algorithm': self.current_algorithm,
                    'message': response.message
                }
        else:
            return {
                'accepted': False,
                'status': 'Error',
                'algorithm': self.current_algorithm,
                'message': 'Service call failed'
            }

app = Flask(__name__)
CORS(app)
ros_client = None

@app.route('/set_algorithm', methods=['POST'])
def set_algorithm_endpoint():
    if ros_client is None:
        return jsonify({
            'accepted': False,
            'status': 'Error',
            'algorithm': 'None',
            'message': 'ROS2 not ready'
        }), 500
    
    try:
        data = request.get_json()
        algorithm = data.get('algorithm', 'None')
        
        if algorithm not in ['LTC', 'MPC', 'Other']:
            return jsonify({
                'accepted': False,
                'status': 'Rejected',
                'algorithm': ros_client.current_algorithm,
                'message': f'Invalid algorithm: {algorithm}'
            }), 400
        
        result = ros_client.call_service(algorithm)
        return jsonify(result)
    except Exception as e:
        return jsonify({
            'accepted': False,
            'status': 'Error',
            'algorithm': ros_client.current_algorithm if ros_client else 'None',
            'message': str(e)
        }), 500

@app.route('/get_algorithm', methods=['GET'])
def get_algorithm_endpoint():
    if ros_client is None:
        return jsonify({
            'algorithm': 'None',
            'status': 'Error'
        }), 500
    
    return jsonify({
        'algorithm': ros_client.current_algorithm,
        'status': ros_client.current_status
    })

def run_flask():
    app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)

def main(args=None):
    global ros_client
    rclpy.init(args=args)
    ros_client = GuidanceAlgorithmClient()
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    print("Guidance Algorithm Bridge is running on http://localhost:5001")
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ros_client)
    executor.spin()
    
    ros_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()