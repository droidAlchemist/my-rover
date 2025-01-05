import os
from glob import glob
from setuptools import setup

package_name = 'iot_controller'

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
    description='ROS2 controller for IOT Core and Kinesis',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_publisher = iot_controller.mqtt_publisher:main',
            'mqtt_listener = iot_controller.mqtt_listener:main',
        ],
    },
)
