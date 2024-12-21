
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from awscrt import mqtt 
from iot_controller.connection_helper import ConnectionHelper

RETRY_WAIT_TIME_SECONDS = 5

class MqttPublisher(Node):
    def __init__(self):
        super().__init__('mqtt_publisher')
        self.declare_parameter("path_for_config", "")
        self.declare_parameter("discover_endpoints", False)

        path_for_config = self.get_parameter("path_for_config").get_parameter_value().string_value
        discover_endpoints = self.get_parameter("discover_endpoints").get_parameter_value().bool_value
        self.connection_helper = ConnectionHelper(self.get_logger(), path_for_config, discover_endpoints)

        self.init_subs()

    def init_subs(self):
        """Subscribe to ros2 ugv topics"""
        self.create_subscription(
            Float32,
            'voltage',
            self.voltage_listener_callback,
            10
        )
         # Create a subscription to the /odom topic to get the odometry data
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Create a subscription to the /robot_pose topic to get the robot's current pose
        self.create_subscription(PoseStamped, '/robot_pose', self.robot_pose_callback, 10) 
        # Create a subscription to the /imu/data topic to get the robot's imu data
        self.create_subscription(
            Imu,
            'imu/data',
            self.robot_imu_callback,
            10
        )

    def publish_message(self, message_json):
        """Callback for the ros2 telemetry topic"""
        self.get_logger().info("Received ROS2 data - {}\nPublishing to AWS IoT".format(message_json))
        self.connection_helper.mqtt_conn.publish(
            topic="ros2_telemetry_topic",
            payload=message_json,
            qos=mqtt.QoS.AT_LEAST_ONCE
        )       

    def voltage_listener_callback(self, msg):
        """Callback for the ros2 voltage topic"""
        self.get_logger().info(msg)
        message_json = {
            "voltage": msg.data
        }
        self.publish_message(message_json)

    def odom_callback(self, msg):
        """Callback for the ros2 odometry topic"""
        self.get_logger().info(msg)
        message_json = {
            "odometry": msg.data
        }
        self.publish_message(message_json)

    def robot_pose_callback(self, msg):
        """Callback for the ros2 robot pose topic"""
        self.get_logger().info(msg)
        message_json = {
            "pose": msg.data
        }
        self.publish_message(message_json)

    def robot_imu_callback(self, msg):
        """Callback for the ros2 robot pose topic"""
        self.get_logger().info(msg)
        message_json = {
            "imu": msg.data
        }
        self.publish_message(message_json)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MqttPublisher()

    rclpy.spin(minimal_subscriber)

    # Destroy the node
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
