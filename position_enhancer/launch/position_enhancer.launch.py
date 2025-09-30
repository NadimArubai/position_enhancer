from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='position_enhancer',
            executable='position_solver_node',
            name='position_enhancer',
            output='screen',
            parameters=[]
        ),
    ])
