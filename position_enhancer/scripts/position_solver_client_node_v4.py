#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from position_enhancer_interfaces.srv import EnhancePosition
from geometry_msgs.msg import Pose, Quaternion
import math
import numpy as np
from concurrent.futures import Future
import threading

class PositionSolverClient(Node):
    def __init__(self):
        super().__init__('position_solver_client')
        self.client = self.create_client(EnhancePosition, '/enhance_position')
        self.pending_requests = {}
        self.request_counter = 0
        
        # Wait for service to be available in background
        self.service_ready = False
        self.service_check_thread = threading.Thread(target=self._wait_for_service_background)
        self.service_check_thread.daemon = True
        self.service_check_thread.start()
    
    def _wait_for_service_background(self, timeout_sec=30.0):
        """Wait for the service to become available in background thread"""
        try:
            if self.client.wait_for_service(timeout_sec=timeout_sec):
                self.service_ready = True
                self.get_logger().info('Service enhance_position is available')
            else:
                self.get_logger().error(f'Service enhance_position not available after {timeout_sec} seconds')
        except Exception as e:
            self.get_logger().error(f'Error waiting for service: {e}')
    
    def is_service_ready(self):
        """Check if service is ready without blocking"""
        return self.service_ready
    
    def wait_for_service_ready(self, timeout_sec=None):
        """Optionally wait for service to be ready (non-blocking if called without timeout)"""
        if timeout_sec is None:
            return self.service_ready
        
        if not self.service_ready:
            self.service_check_thread.join(timeout=timeout_sec)
        return self.service_ready
        
    def create_pose(self, x, y, yaw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)
        return pose
    
    def solve_object_position_async(self, observations, request_id=None):
        """
        Non-blocking version - sends request and returns immediately with Future
        
        observations: list of tuples (robot_x, robot_y, robot_yaw, range, bearing, range_unc, bearing_unc)
        request_id: optional identifier for this request. If None, auto-generated.
        
        Returns: (request_id, future) where future will contain the result when ready
        """
        if not self.service_ready:
            self.get_logger().warn('Service not ready yet')
            future = Future()
            future.set_exception(RuntimeError('Service not available'))
            return None, future
        
        if not observations:
            self.get_logger().warn('No observations provided')
            future = Future()
            future.set_result(None)
            return None, future
        
        if len(observations) < 2:
            self.get_logger().warn('At least 2 observations recommended for triangulation')
        
        # Generate request ID if not provided
        if request_id is None:
            self.request_counter += 1
            request_id = f"req_{self.request_counter}"
        
        # Create and send request
        request = EnhancePosition.Request()
        
        for obs in observations:
            x, y, yaw, r, b, r_unc, b_unc = obs
            request.robot_poses.append(self.create_pose(x, y, yaw))
            request.ranges.append(r)
            request.bearings.append(b)
            request.range_uncertainties.append(r_unc)
            request.bearing_uncertainties.append(b_unc)
        
        # Create future for this request
        future = Future()
        self.pending_requests[request_id] = future
        
        # Send async request
        service_future = self.client.call_async(request)
        service_future.add_done_callback(
            lambda fut, req_id=request_id: self._service_response_callback(fut, req_id)
        )
        
        self.get_logger().info(f'Sent async position solve request: {request_id}')
        return request_id, future
    
    def _service_response_callback(self, future, request_id):
        """Handle service response and complete the user-facing future"""
        try:
            if future.done():
                response = future.result()
                user_future = self.pending_requests.pop(request_id, None)
                if user_future and not user_future.done():
                    user_future.set_result(response)
                    self.get_logger().info(f'Request {request_id} completed successfully')
            else:
                raise Exception("Service future not done")
        except Exception as e:
            user_future = self.pending_requests.pop(request_id, None)
            if user_future and not user_future.done():
                user_future.set_exception(e)
            self.get_logger().error(f'Request {request_id} failed: {e}')
    
    def cancel_request(self, request_id):
        """Cancel a pending request"""
        if request_id in self.pending_requests:
            future = self.pending_requests[request_id]
            if not future.done():
                future.cancel()
            del self.pending_requests[request_id]
            self.get_logger().info(f'Cancelled request: {request_id}')
            return True
        return False
    
    def get_pending_requests(self):
        """Get list of pending request IDs"""
        return list(self.pending_requests.keys())

# Example usage with non-blocking pattern
def main():
    print('Starting non-blocking position solver client...')
    rclpy.init()
    
    try:
        client = PositionSolverClient()
        
        # Wait a bit for service discovery (non-blocking check)
        import time
        max_wait = 10.0
        start_time = time.time()
        while not client.is_service_ready() and (time.time() - start_time) < max_wait:
            print('Waiting for service...')
            time.sleep(0.5)
        
        if not client.is_service_ready():
            print("Service not available, but continuing with example...")
        
        # Example: triangulate object from multiple robot poses
        observations = [
            # (x, y, yaw, range, bearing, range_unc, bearing_unc)
            (0.0, 0.0, 0.0, 5.0, 0.5, 0.1, 0.05),
            (3.0, 1.0, math.pi/2, 4.0, -0.3, 0.1, 0.05),
            (1.0, 4.0, math.pi, 6.0, 0.2, 0.1, 0.05)
        ]
        
        print('Sending async enhance_position request...')
        request_id, future = client.solve_object_position_async(observations)
        
        # Do other work while request is processing...
        print('Request sent, doing other work...')
        for i in range(5):
            print(f'Working... {i+1}/5')
            time.sleep(0.5)
        
        # Check if result is ready (non-blocking check)
        if future.done():
            try:
                result = future.result()
                if result and result.success:
                    print(f"Enhanced position: ({result.enhanced_pose.position.x:.3f}, "
                          f"{result.enhanced_pose.position.y:.3f})")
                    print(f"Uncertainty: (±{result.pose_uncertainty_x:.3f}, "
                          f"±{result.pose_uncertainty_y:.3f})")
                else:
                    error_msg = result.message if result else "Unknown error"
                    print(f"Error: {error_msg}")
            except Exception as e:
                print(f"Request failed: {e}")
        else:
            print("Request still processing, you can continue working...")
            # Optionally wait for completion later
            # result = future.result(timeout=5.0)  # This would block with timeout
            
    except Exception as e:
        print(f"Failed to initialize client: {e}")
    finally:
        rclpy.shutdown()
        print('Done')

if __name__ == '__main__':
    main()
