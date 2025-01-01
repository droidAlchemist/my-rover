
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu
from awscrt import mqtt5
import json 
import serial
from iot_controller.connection_helper import ConnectionHelper

RETRY_WAIT_TIME_SECONDS = 100
TELEMETRY_MESSAGE_TOPIC = "ros2/telemetry/topic"

# Initialize serial communication with the UGV
serial_port = '/dev/ttyAMA0'
ser = serial.Serial(serial_port, 115200, timeout=1)

def todict(obj):        
    if isinstance(obj, dict):
        return dict((key.lstrip("_"), todict(val)) for key, val in obj.items())
    elif hasattr(obj, "_ast"):
        return todict(obj._ast())
    elif hasattr(obj, "__iter__") and not isinstance(obj, str):
        return [todict(v) for v in obj]
    elif hasattr(obj, '__dict__'):
        return todict(vars(obj))
    elif hasattr(obj, '__slots__'):
        return todict(dict((name, getattr(obj, name)) for name in getattr(obj, '__slots__')))
    return obj

class MqttPublisher(Node):
    def __init__(self):
        super().__init__('mqtt_publisher')
        self.declare_parameter("path_for_config", "")
        self.declare_parameter("discover_endpoints", False)

        path_for_config = self.get_parameter("path_for_config").get_parameter_value().string_value
        self.connection_helper = ConnectionHelper(self.get_logger(), path_for_config)

        self.init_subs()
        self.init_rover()

    def init_rover(self):
        """Setup to UGV Rover to send telemetry"""
        ctrl_data = json.dumps({
            "T":900,
            "main": 1,
            "module": 0
        }) + "\n"     
        ser.write(ctrl_data.encode())    

        ctrl_data = json.dumps({"T":131,"cmd":0}) + "\n"            
        ser.write(ctrl_data.encode())  

        ctrl_data = json.dumps({"T":131,"cmd":1}) + "\n"            
        ser.write(ctrl_data.encode()) 

        #close serial comminication
        ser.close()

    def init_subs(self):
        """Subscribe to ros2 topics"""
        # Create a subscription to the /voltage topic to get the voltage data
        self.create_subscription(
            Float32,
            '/voltage',
            self.voltage_listener_callback,
            10
        )
        # Create a subscription to the /temperature topic to get the temperature data
        self.create_subscription(
            Float32,
            '/temperature',
            self.temperature_listener_callback,
            10
        )
        # Create a subscription to the /odom topic to get the odometry data
        self.create_subscription(Float32MultiArray, '/odom/odom_raw', self.odom_callback, 10)
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
        publish_future = self.connection_helper.client.publish(mqtt5.PublishPacket(
            topic=TELEMETRY_MESSAGE_TOPIC,
            payload=json.dumps(message_json),
            qos=mqtt5.QoS.AT_LEAST_ONCE
        ))

        publish_completion_data = publish_future.result(RETRY_WAIT_TIME_SECONDS)
        print("PubAck received with {}".format(repr(publish_completion_data.puback.reason_code)))

    def voltage_listener_callback(self, msg):
        """Callback for the ros2 voltage topic"""
        message_json = {
            "type": "voltage",
            "data": msg.data
        }
        self.publish_message(message_json)

    def temperature_listener_callback(self, msg):
        """Callback for the ros2 temperature topic"""
        message_json = {
            "type": "temperature",
            "data": msg.data
        }
        self.publish_message(message_json)

    def odom_callback(self, msg):
        """Callback for the ros2 odometry topic"""
        try:        
            message_json = {
                "type": "odometry",
                "data": msg.data
            }
            self.publish_message(message_json)
        except:
           self.get_logger().info("Failed to send odometry data")

    def robot_imu_callback(self, msg):
        """Callback for the ros2 robot pose topic"""
        try:
            message_json = {
                "type": "imu",
                "data": todict(msg.data)
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
