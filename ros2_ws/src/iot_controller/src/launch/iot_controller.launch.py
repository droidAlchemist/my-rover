from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Define the nodes to be launched
    iot_telemetry_node = Node(
        package='iot_controller',
        executable='mqtt_publisher'
    )

    iot_controller_node = Node(
        package='iot_controller',
        executable='mqtt_listener'
    )

    # Return the launch description with all defined actions
    return LaunchDescription([
        iot_telemetry_node,
        iot_controller_node
    ])