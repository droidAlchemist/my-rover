
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from awscrt import mqtt5 
from iot_controller.connection_helper import ConnectionHelper
from iot_controller.kinesis.kinesis_connection_helper import KinesisConnectionHelper
import json
import iot_controller.my_constants as my_constants

class MqttListener(Node):
    def __init__(self):
        super().__init__('mqtt_listener')
        self.connection_helper = ConnectionHelper(self.get_logger(), self.on_message_received)
        # Initialize subscription
        self.init_subs()
        # Initialize Kinesis
        self.init_kinesis()

    def init_kinesis(self):
        """Setup kinesis web rtc connection"""
        self.kvs = KinesisConnectionHelper()
        self.is_camera_on = False        

    def init_subs(self):
        """Subscribe to AWS IOT topics"""
        self.get_logger().info("Subscribing to AWS IOT Core topics")
        subscribe_future = self.connection_helper.client.subscribe(subscribe_packet=mqtt5.SubscribePacket(
            subscriptions=[
                    mqtt5.Subscription(
                    topic_filter=my_constants.SUBSCRIBE_IOT_CONTROL_TOPIC,
                    qos=mqtt5.QoS.AT_MOST_ONCE),
                    mqtt5.Subscription(
                    topic_filter=my_constants.SUBSCRIBE_CAMERA_TOPIC,
                    qos=mqtt5.QoS.AT_MOST_ONCE)
                ]
        ))
        suback = subscribe_future.result(my_constants.WAIT_TIME_SECONDS)
        self.get_logger().info("Subscribed with {}".format(suback.reason_codes))

    def send_velocity(self, commandDict):
        """Send twist message to ros2 cmd_vel topic"""
        x = commandDict["linear"]["x"]
        z = commandDict["angular"]["z"]
        # Publish twist message
        twist=Twist()
        twist.linear.x = float(x)
        twist.angular.z = float(z)
        vel_pub.publish(twist)

    def send_camera_action(self, commandDict):
        """Start/Stop streaming of camera using Kinesis WebRTC"""
        action = commandDict["action"]
        if action == "start":
            self.get_logger().info("Start streaming camera")
            self.is_camera_on = True
            self.kvs.start()
        else:
            self.get_logger().info("Stop streaming camera")
            self.is_camera_on = False
            self.kvs.stop()

    def on_message_received(self, payload, topic):
        self.get_logger().info("Received message from topic '{}': {}".format(topic, payload))
        command = payload.decode('utf8').replace("'", '"')
        try:
            commandDict = json.loads(command)
            if topic == my_constants.SUBSCRIBE_IOT_CONTROL_TOPIC:
                self.send_velocity(commandDict)
            elif topic == my_constants.SUBSCRIBE_CAMERA_TOPIC:
                self.send_camera_action(commandDict)
        except:
            self.get_logger().info("Error parsing message from topic {}: {}".format(topic, command))


def main(args=None):
    rclpy.init(args=args)
    global vel_pub

    try:
        vel_pub = rclpy.create_node('vel_publisher').create_publisher(Twist, my_constants.ROS2_CMD_VEL_TOPIC, 10)
        minimal_subscriber = MqttListener()
        # Spin nodes
        rclpy.spin(minimal_subscriber)
        rclpy.spin(vel_pub)
    except KeyboardInterrupt:
        pass
    finally:
        # Do cleanup
        minimal_subscriber.connection_helper.cleanup()
        if minimal_subscriber.is_camera_on:
            minimal_subscriber.kvs.stop()
    
    # Destroy the node
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
