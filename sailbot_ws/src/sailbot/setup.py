from setuptools import setup
from glob import glob
#     packages=[package_name, 'sailbot.autonomous', 'sailbot.onBoardDisplay', 'sailbot.onBoardDisplay.lib'],
package_name = 'sailbot'
epd_library = 'onBoardDisplay.lib'
autonomous_pkg = 'autonomous'
obd_pkg = 'onBoardDisplay'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, autonomous_pkg, obd_pkg, epd_library],
    package_data = {
        epd_library: ["*.so"], # needed for e-Paper SPI library sysfs_software_spi.so TODO: use native spi lib
    },
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
            'obd_controller = '+ obd_pkg +'.obd_controller:main',
            'test_pub = '+ obd_pkg +'.test_pub:main',
        ],
    },
)
