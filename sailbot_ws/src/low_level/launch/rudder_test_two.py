from launch import LaunchDescription
from launch_ros.actions import Node

# Desrciption:
# Tests the rudders using the discontinuous fuzzy logic system


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='low_level',
            node_executable='rudder_control_continuous',
            name='rudder_control'
        ),
        Node(
            package='low_level',
            node_executable='desired_heading_publisher',
            name='d_heading',
            outout='screen'
        ),
        Node(
            package='sailbot',
            node_executable='pwm_controller',
            name='pwm'
        ),
        Node(
            package='sailbot',
            node_executable='airmar_reader',
            name='airmar'
        )
    ])
