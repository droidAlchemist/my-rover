
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import json
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
        """Subscribe to ros2 topics"""
        self.create_subscription(
            Float32,
            '/voltage',
            self.voltage_listener_callback,
            10
        )
         # Create a subscription to the /odom topic to get the odometry data
        self.create_subscription(Odometry, '/odom/odom_raw', self.odom_callback, 10)
        # Create a subscription to the /imu/data topic to get the robot's imu data
        self.create_subscription(
            Imu,
            'imu/data_raw',
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
        message_json = {
            "voltage": msg.data
        }
        self.publish_message(message_json)

    def odom_callback(self, msg):
        """Callback for the ros2 odometry topic"""
        try:        
            twist = msg.twist.twist
            # velocity = twist.linear.x
            # angular_velocity = twist.angular.z
            pose = msg.pose.pose
            x = pose.orientation.x
            y = pose.orientation.y
            distance = pose.position
            curr_time = msg.header.stamp            
            message_json = {
                "odometry": {
                    "pose": {
                        "x": x,
                        "y": y,
                        "distance": distance
                    },
                    "twist": twist,
                    "time": curr_time
                }
            }
            self.publish_message(message_json)
        except:
           self.get_logger().info("Failed to send odometry data")

    def robot_imu_callback(self, msg):
        """Callback for the ros2 robot pose topic"""
        try:
            message_json = {
                "imu": json.dumps(msg.data)
            }
            self.publish_message(message_json)
        except:
           self.get_logger().info("Failed to send IMU data") 
        

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MqttPublisher()
    rclpy.spin(minimal_subscriber)

    # Destroy the node
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
