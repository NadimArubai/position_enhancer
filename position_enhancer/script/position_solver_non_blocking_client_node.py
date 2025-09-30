#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gtsam_msgs.srv import EnhancePosition
from geometry_msgs.msg import Pose, Quaternion
import math
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

class PositionSolverClient(Node):
    def __init__(self):
        super().__init__('position_solver_client')
        
        # Use ReentrantCallbackGroup to allow multiple service calls
        self.callback_group = ReentrantCallbackGroup()
        
        self.client = self.create_client(
            EnhancePosition, 
            '/enhance_position',
            callback_group=self.callback_group
        )
        
        self.service_available = False
        self.check_service_timer = self.create_timer(
            5.0,  # Check every second
            self.check_service_availability,
            callback_group=self.callback_group
        )
        
        self.active_requests = {}  # Track active requests by future
#        self.wait_for_service()
    
    def wait_for_service(self, timeout_sec=10.0):
        """Wait for the service to become available"""
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'Service enhance_position not available after {timeout_sec} seconds')
            raise RuntimeError('Service not available')
        self.get_logger().info('Service enhance_position is available')
        
        
    def check_service_availability(self):
        """Periodically check if service is available"""
        if self.client.service_is_ready():
            if not self.service_available:
                self.get_logger().info('Service enhance_position is available')
                self.service_available = True
        else:
            if self.service_available:
                self.get_logger().warn('Service enhance_position became unavailable')
                self.service_available = False
    
    def create_pose(self, x, y, yaw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)
        return pose
    
    def solve_object_position_async(self, observations, callback=None, request_id=None):
        """
        Non-blocking method to solve object position
        
        Args:
            observations: list of tuples (robot_x, robot_y, robot_yaw, range, bearing, range_unc, bearing_unc)
            callback: function to call when result is ready (signature: callback(result, request_id))
            request_id: optional identifier for this request
        
        Returns:
            future: Future object that can be used to check status or add callbacks
        """
        if not observations:
            self.get_logger().warn('No observations provided')
            if callback:
                callback(None, request_id)
            return None
        
        if len(observations) < 2:
            self.get_logger().warn('At least 2 observations recommended for triangulation')
        
        request = EnhancePosition.Request()
        
        for obs in observations:
            x, y, yaw, r, b, r_unc, b_unc = obs
            request.robot_poses.append(self.create_pose(x, y, yaw))
            request.ranges.append(r)
            request.bearings.append(b)
            request.range_uncertainties.append(r_unc)
            request.bearing_uncertainties.append(b_unc)
        
        if not self.service_available:
            self.get_logger().warn('Service not available, request will be queued')
        
        future = self.client.call_async(request)
        
        # Store callback and request_id with the future
        future._callback_data = {
            'callback': callback,
            'request_id': request_id,
            'timestamp': self.get_clock().now()
        }
        
        # Add done callback to handle the response
        future.add_done_callback(self._handle_service_response)
        
        # Track active requests
        self.active_requests[future] = future._callback_data
        
        return future
    
    def _handle_service_response(self, future):
        """Handle the service response when it completes"""
        try:
            # Remove from active requests
            callback_data = self.active_requests.pop(future, None)
            
            if not callback_data:
                self.get_logger().warn('Received response for unknown request')
                return
            
            if future.exception() is not None:
                self.get_logger().error(f'Service call failed: {future.exception()}')
                if callback_data['callback']:
                    callback_data['callback'](None, callback_data['request_id'])
                return
            
            result = future.result()
            if callback_data['callback']:
                callback_data['callback'](result, callback_data['request_id'])
                
        except Exception as e:
            self.get_logger().error(f'Error handling service response: {e}')
    
    def cancel_request(self, future):
        """Cancel a specific request"""
        if future in self.active_requests:
            # ROS 2 doesn't support canceling service calls directly,
            # but we can remove the callback and mark it as handled
            if future in self.active_requests:
                del self.active_requests[future]
            self.get_logger().info('Request canceled')
    
    def cancel_all_requests(self):
        """Cancel all pending requests"""
        for future in list(self.active_requests.keys()):
            self.cancel_request(future)
        self.get_logger().info('All requests canceled')
    
    def get_active_requests_count(self):
        """Get number of active requests"""
        return len(self.active_requests)

# Example usage with non-blocking calls
def main():
    print('Starting non-blocking position solver client...')
    rclpy.init()
    
    # Use multi-threaded executor for concurrent processing
    executor = MultiThreadedExecutor()
    
    try:
        client = PositionSolverClient()
        executor.add_node(client)
        
        # Define callback function
        def result_callback(result, request_id):
            if result and result.success:
                print(f"[{request_id}] Enhanced position: ({result.enhanced_pose.position.x:.3f}, "
                      f"{result.enhanced_pose.position.y:.3f})")
                print(f"[{request_id}] Uncertainty: (±{result.pose_uncertainty_x:.3f}, "
                      f"±{result.pose_uncertainty_y:.3f})")
            else:
                error_msg = result.message if result else "Unknown error"
                print(f"[{request_id}] Error: {error_msg}")
        
        # Example: make multiple non-blocking calls
        observation_sets = [
            {
                'id': 'request_1',
                'data': [
                    (0.0, 0.0, 0.0, 5.0, 0.5, 0.1, 0.05),
                    (3.0, 1.0, math.pi/2, 4.0, -0.3, 0.1, 0.05),
                ]
            },
            {
                'id': 'request_2', 
                'data': [
                    (1.0, 4.0, math.pi, 6.0, 0.2, 0.1, 0.05),
                    (2.0, 2.0, math.pi/4, 3.0, 0.1, 0.1, 0.05),
                ]
            }
        ]
        
        print('Making non-blocking service calls...')
        futures = []
        for obs_set in observation_sets:
            future = client.solve_object_position_async(
                obs_set['data'], 
                callback=result_callback,
                request_id=obs_set['id']
            )
            futures.append(future)
            print(f"Sent request: {obs_set['id']}")
        
        # Continue doing other work while waiting for responses
        print('Main thread can continue working...')
        
        # Simulate some other work
        import time
        for i in range(5):
            print(f"Doing other work... {i+1}/5")
            time.sleep(1)
            executor.spin_once(timeout_sec=0.1)
        
        # Wait for all requests to complete with timeout
        print('Waiting for remaining responses...')
        start_time = time.time()
        timeout = 10.0
        
        while client.get_active_requests_count() > 0 and (time.time() - start_time) < timeout:
            executor.spin_once(timeout_sec=0.1)
            time.sleep(0.1)
        
        if client.get_active_requests_count() > 0:
            print('Timeout reached, canceling remaining requests...')
            client.cancel_all_requests()
            
    except Exception as e:
        print(f"Failed to initialize client: {e}")
    finally:
        client.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        print('Done')

if __name__ == '__main__':
    main()
