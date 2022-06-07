from launch import LaunchDescription
from launch_ros.actions import Node

# Desrciption:
# Tests the rudders using the discontinuous fuzzy logic system


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sailbot',
            node_executable='airmar_reader',
            name='airmar'
        ),
        Node(
            package='sailbot',
            node_executable='pwm_controller',
            name='pwm'
        ),
        Node(
            package='sailbot',
            node_executable='trim_tab_comms',
            name='trim_tab_comms'
        ),
        Node(
            package='low_level',
            node_executable='station_target_gen',
            name='station_target_gen'
        ),
        Node(
            package='low_level',
            node_executable='tt_state_pub',
            name='tt_state_pub'
        ),
        Node(
            package='low_level',
            node_executable='rudder_control',
            name='rudder_control'
        ),
        Node(
            package='low_level',
            node_executable='point2point',
            name='point2point'
        )
    ])