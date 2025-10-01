#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from position_enhancer_interfaces.srv import EnhancePosition
from geometry_msgs.msg import Pose, Quaternion
import math
import time

class PositionSolverClient(Node):
    def __init__(self):
        super().__init__('position_solver_client')
        
        # Debug: Print node info
        self.get_logger().info(f"Node name: {self.get_name()}")
        self.get_logger().info(f"Namespace: {self.get_namespace()}")
        
        # Try different service name patterns
        service_names_to_try = [
            'enhance_position',
            '/enhance_position',
            # Add any specific namespace if you know it
        ]
        
        self.client = None
        for service_name in service_names_to_try:
            try:
                self.client = self.create_client(EnhancePosition, service_name)
                self.get_logger().info(f"Trying service name: {service_name}")
                if self.wait_for_service(timeout_sec=2.0):
                    self.get_logger().info(f"Connected to service: {service_name}")
                    break
                else:
                    self.destroy_client(self.client)
                    self.client = None
            except Exception as e:
                self.get_logger().warn(f"Failed with {service_name}: {e}")
        
        if self.client is None:
            self.get_logger().error("Could not connect to any service variant")
            self.list_all_services()
    
    def list_all_services(self):
        """List all available services for debugging"""
        try:
            services = self.get_service_names_and_types()
            self.get_logger().info("=== ALL AVAILABLE SERVICES ===")
            for name, types in services:
                if 'EnhancePosition' in str(types):
                    self.get_logger().info(f"MATCH: {name} -> {types}")
                else:
                    self.get_logger().info(f"  {name} -> {types}")
        except Exception as e:
            self.get_logger().error(f"Failed to list services: {e}")
    
    def wait_for_service(self, timeout_sec=10.0):
        """Wait for service with better debugging"""
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.client.service_is_ready():
                return True
            self.get_logger().info('Waiting for service...')
            time.sleep(1.0)
        return False
    
    # ... rest of your methods unchanged ...

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


def main():
    print('Starting position solver client with debugging...')
    rclpy.init()
    
    try:
        client = PositionSolverClient()
        
        if client.client is None or not client.client.service_is_ready():
            print("Service not available. Check the debug output above.")
            return
        
        # Your test code here
        observations = [
            (0.0, 0.0, 0.0, 5.0, 0.5, 0.1, 0.05),
            (3.0, 1.0, math.pi/2, 4.0, -0.3, 0.1, 0.05),
        ]
        
        result = client.solve_object_position(observations)
        if result:
            print(f"Success: {result}")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
