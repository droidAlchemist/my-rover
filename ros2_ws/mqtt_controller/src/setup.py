import os
from glob import glob
from setuptools import setup

package_name = 'my_rover'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua SJ',
    maintainer_email='joshua.sj7@gmail.com',
    description='AWS IoT connectivity demo for ROS2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_telemetry_pub = my_rover.mock_telemetry_publisher:main',
            'mqtt_telemetry_pub = my_rover.mqtt_telemetry_publisher:main',
            'mqtt_control_sub = my_rover.mqtt_control_listener:main',
        ],
    },
)
