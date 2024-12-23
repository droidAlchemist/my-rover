
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from awscrt import mqtt 
from iot_controller.connection_helper import ConnectionHelper
import json

RETRY_WAIT_TIME_SECONDS = 5

class MqttControlListener(Node):
    def __init__(self):
        super().__init__('mqtt_control_listener')
        self.declare_parameter("path_for_config", "")
        self.declare_parameter("discover_endpoints", False)

        path_for_config = self.get_parameter("path_for_config").get_parameter_value().string_value
        discover_endpoints = self.get_parameter("discover_endpoints").get_parameter_value().bool_value
        self.connection_helper = ConnectionHelper(self.get_logger(), path_for_config, discover_endpoints)

        self.init_subs()

    def init_subs(self):
        # """Subscribe to AWS IOT control topic"""
        self.get_logger().info("Subscribing to AWS IoT core robot control topic cmd_vel")
        self.connection_helper.mqtt_conn.subscribe(
            "cmd_vel",
            mqtt.QoS.AT_LEAST_ONCE,
            self.on_message_received
        )

    def on_message_received(self, topic, payload, **kwargs):
        print("Received message from topic '{}': {}".format(topic, payload))
        command = payload.decode('utf8').replace("'", '"')
        try:
            commandDict = json.loads(command)
            x = commandDict["linear"]["x"]
            z = commandDict["angular"]["z"]
            # Publish twist message
            twist=Twist()
            twist.linear.x = float(x)
            twist.angular.z = float(z)
            vel_pub.publish(twist)
        except:
            print("Error parsing message from topic '{}': {}".format(topic, payload))



def main(args=None):
    rclpy.init(args=args)
    global vel_pub
    vel_pub = rclpy.create_node('vel_publisher').create_publisher(Twist, "cmd_vel", 10)
    minimal_subscriber = MqttControlListener()
    
    rclpy.spin(minimal_subscriber)
    rclpy.spin(vel_pub)

    # Destroy the node
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
