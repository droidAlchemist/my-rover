
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from awscrt import mqtt5 
from iot_controller.connection_helper import ConnectionHelper
import json

RETRY_WAIT_TIME_SECONDS = 100
SUBSCRIBE_IOT_CONTROL_TOPIC = "cmd_vel"

class MqttControlListener(Node):
    def __init__(self):
        super().__init__('mqtt_control_listener')
        self.declare_parameter("path_for_config", "")
        self.declare_parameter("discover_endpoints", False)

        path_for_config = self.get_parameter("path_for_config").get_parameter_value().string_value
        self.connection_helper = ConnectionHelper(self.get_logger(), path_for_config)
        self.connection_helper.send_twist_message = self.on_message_received

        self.init_subs()

    def init_subs(self):
        """Subscribe to AWS IOT control topic"""
        self.get_logger().info("Subscribing to AWS IoT core robot control topic cmd_vel")
        # self.connection_helper.mqtt_conn.subscribe(
        #     SUBSCRIBE_IOT_CONTROL_TOPIC,
        #     mqtt.QoS.AT_LEAST_ONCE,
        #     self.on_message_received
        # )
        subscribe_future = self.connection_helper.client.subscribe(subscribe_packet=mqtt5.SubscribePacket(
        subscriptions=[mqtt5.Subscription(
            topic_filter=SUBSCRIBE_IOT_CONTROL_TOPIC,
            qos=mqtt5.QoS.AT_LEAST_ONCE)]
        ))
        suback = subscribe_future.result(RETRY_WAIT_TIME_SECONDS)
        self.logger.info("Subscribed with {}".format(suback.reason_codes))

    def on_message_received(self, payload):
        self.logger.info("Received message from topic: {}".format(payload))
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
            self.logger.info("Error parsing message from topic : {}".format(payload))



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
