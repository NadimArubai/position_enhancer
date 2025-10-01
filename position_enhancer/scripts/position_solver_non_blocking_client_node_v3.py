#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from position_enhancer_interfaces.srv import EnhancePosition
from geometry_msgs.msg import Pose, Quaternion
import math

class PositionSolverClient(Node):
    def __init__(self):
        super().__init__('position_solver_client')
        
        self.client = self.create_client(EnhancePosition, '/enhance_position')
        self.service_available = False
        self.active_request = None
        
        # Check service availability
        self.check_service_timer = self.create_timer(5.0, self.check_service_availability)
    
    def check_service_availability(self):
        """Check if service is available"""
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
    
    def solve_object_position_async(self, observations, callback=None):
        """
        Non-blocking method to solve object position - handles one request at a time
        
        Args:
            observations: list of tuples (robot_x, robot_y, robot_yaw, range, bearing, range_unc, bearing_unc)
            callback: function to call when result is ready
        
        Returns:
            bool: True if request was sent, False if busy
        """
        if self.active_request is not None:
            self.get_logger().warn('Service busy - only one request allowed at a time')
            return False
        
        if not observations:
            self.get_logger().warn('No observations provided')
            return False
        
        request = EnhancePosition.Request()
        
        for obs in observations:
            x, y, yaw, r, b, r_unc, b_unc = obs
            request.robot_poses.append(self.create_pose(x, y, yaw))
            request.ranges.append(r)
            request.bearings.append(b)
            request.range_uncertainties.append(r_unc)
            request.bearing_uncertainties.append(b_unc)
        
        future = self.client.call_async(request)
        self.active_request = future
        
        # Store callback
        future._callback = callback
        
        # Add done callback
        future.add_done_callback(self._handle_service_response)
        
        self.get_logger().info('Sent position solve request')
        return True
    
    def _handle_service_response(self, future):
        """Handle the service response when it completes"""
        self.active_request = None  # Clear active request
        
        callback = getattr(future, '_callback', None)
        
        if future.exception() is not None:
            self.get_logger().error(f'Service call failed: {future.exception()}')
            if callback:
                callback(None)
            return
        
        result = future.result()
        if callback:
            callback(result)
    
    def is_busy(self):
        """Check if currently processing a request"""
        return self.active_request is not None

# Example usage
def main():
    rclpy.init()
    
    client = PositionSolverClient()
    
    def result_callback(result):
        if result and result.success:
            print(f"Enhanced position: ({result.enhanced_pose.position.x:.3f}, "
                  f"{result.enhanced_pose.position.y:.3f})")
            print(f"Uncertainty: (±{result.pose_uncertainty_x:.3f}, "
                  f"±{result.pose_uncertainty_y:.3f})")
        else:
            error_msg = result.message if result else "Unknown error"
            print(f"Error: {error_msg}")
    
    # Example observations
    observations = [
        (0.0, 0.0, 0.0, 5.0, 0.5, 0.1, 0.05),
        (3.0, 1.0, math.pi/2, 4.0, -0.3, 0.1, 0.05),
    ]
    
    # Non-blocking call
    print("Sending request...")
    success = client.solve_object_position_async(observations, result_callback)
    
    if success:
        print("Request sent successfully - main thread continues working...")
        
        # Do other work while waiting for response
        import time
        for i in range(5):
            print(f"Doing other work... {i+1}/5")
            time.sleep(1)
            rclpy.spin_once(client, timeout_sec=0.1)
            
        # Wait for completion
        while client.is_busy():
            print("Waiting for response...")
            rclpy.spin_once(client, timeout_sec=0.1)
            time.sleep(0.1)
            
        print("Done!")
    else:
        print("Failed to send request - service busy")
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
