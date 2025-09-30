#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gtsam_msgs.srv import EnhancePosition
from geometry_msgs.msg import Pose, Quaternion
import math
import numpy as np

class PositionSolverClient(Node):
    def __init__(self):
        super().__init__('position_solver_client')
        self.client = self.create_client(EnhancePosition, '/enhance_position')
        
        # Wait for service to be available
        self.wait_for_service()
    
    def wait_for_service(self, timeout_sec=10.0):
        """Wait for the service to become available"""
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'Service enhance_position not available after {timeout_sec} seconds')
            raise RuntimeError('Service not available')
        self.get_logger().info('Service enhance_position is available')
        
    def create_pose(self, x, y, yaw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)
        return pose
    
    def solve_object_position(self, observations, timeout_sec=10.0):
        """
        observations: list of tuples (robot_x, robot_y, robot_yaw, range, bearing, range_unc, bearing_unc)
        """
        if not observations:
            self.get_logger().warn('No observations provided')
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
        
        try:
            self.wait_for_service()
            self.future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, self.future, timeout_sec=timeout_sec)
            
            if not self.future.done():
                self.get_logger().error('Service call timed out')
                return None
                
            result = self.future.result()
            if result is not None:
                return result
            else:
                self.get_logger().error('Service call failed - no result')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return None

# Example usage
def main():
    print('Starting position solver client...')
    rclpy.init()
    
    try:
        client = PositionSolverClient()
        
        # Example: triangulate object from multiple robot poses
        observations = [
            # (x, y, yaw, range, bearing, range_unc, bearing_unc)
            (0.0, 0.0, 0.0, 5.0, 0.5, 0.1, 0.05),
            (3.0, 1.0, math.pi/2, 4.0, -0.3, 0.1, 0.05),
            (1.0, 4.0, math.pi, 6.0, 0.2, 0.1, 0.05)
        ]
        
        print('Calling enhance_position service...')
        result = client.solve_object_position(observations)
        
        if result and result.success:
            print(f"Enhanced position: ({result.enhanced_pose.position.x:.3f}, "
                  f"{result.enhanced_pose.position.y:.3f})")
            print(f"Uncertainty: (±{result.pose_uncertainty_x:.3f}, "
                  f"±{result.pose_uncertainty_y:.3f})")
        else:
            error_msg = result.message if result else "Unknown error"
            print(f"Error: {error_msg}")
            
    except Exception as e:
        print(f"Failed to initialize client: {e}")
    finally:
        rclpy.shutdown()
        print('Done')

if __name__ == '__main__':
    main()
