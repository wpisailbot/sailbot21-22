from setuptools import setup
from glob import glob

package_name = 'sailbot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, 'sailbot.autonomous'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Worcester Polytechnic Institute',
    maintainer_email='gr-sailbot2122@wpi.edu',
    description='Primary ROS package for WPI\'s autonomous sailboat MQP',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'airmar_reader = sailbot.airmar_reader:main',
            'pwm_controller = sailbot.pwm_controller:main',
            'serial_rc_receiver = sailbot.serial_rc_receiver:main',
            'control_system = sailbot.control_system:main',
            'trim_tab_comms = sailbot.trim_tab_comms:main',
            'debug_interface = sailbot.debug_interface:main',
            'battery_monitor = sailbot.battery_monitor:main',
        ],
    },
)
