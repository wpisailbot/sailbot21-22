from setuptools import setup
from glob import glob

package_name = 'low_level'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrew',
    maintainer_email='andrewdel82@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rudder_test = low_level.rudder_test:main',
            'desired_heading = low_level.desired_heading_publisher:main',
            'rudder_control = low_level.rudder_control:main',
            'point2point = low_level.point_to_point:main',
            'target_location = low_level.lat_long_publisher:main',
            'tt_state_pub = low_level.standalone_trim_tab_state_publisher:main'
        ],
    },
)
