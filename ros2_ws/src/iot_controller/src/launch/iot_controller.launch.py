from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    iot_mqtt_arg = DeclareLaunchArgument(
        'path_for_config', default_value='$IOT_CONFIG_FILE',
        description='Pass the certificate config path'
    )

    # Define the nodes to be launched
    iot_telemetry_node = Node(
        package='iot_controller',
        executable='mqtt_telemetry_pub',
    )

    iot_controller_node = Node(
        package='iot_controller',
        executable='mqtt_control_sub',
    )

    # Return the launch description with all defined actions
    return LaunchDescription([
        iot_mqtt_arg,
        iot_telemetry_node,
        iot_controller_node
    ])