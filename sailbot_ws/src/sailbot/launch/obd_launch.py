from launch import LaunchDescription
from launch_ros.actions import Node

# Description:
# A launch file that simply launches all nodes

#            cwd='../../../install/sailbot/lib/python3.6/site-packages/sailbot/onBoardDisplay',

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='sailbot',
			node_executable='test_pub',
			name='test_pub',
		),
        Node(
			package='sailbot',
			node_executable='obd_controller',
			name='obd_controller',
		)

    ])