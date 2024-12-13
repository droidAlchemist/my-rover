import rclpy
import json
import random
from rclpy.node import Node
from std_msgs.msg import String


class MockTelemetryPublisher(Node):

    def __init__(self):
        super().__init__('mock_telemetry_publisher')
        self.publisher_ = self.create_publisher(
                String,
                'mock_telemetry',
                10)
        timer_period = 15  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()

        # Create data and publish mock telemetry data
        mock_telemetry = {}
        mock_telemetry["battery"] = round(random.uniform(15, 90), 2)
        mock_telemetry["velocity"] = round(random.uniform(3, 40), 2)
        mock_telemetry["temperature"] = round(random.uniform(23, 40), 2)
        mock_telemetry["distance"] = round(random.uniform(20, 60), 2)
        msg.data = json.dumps(mock_telemetry)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MockTelemetryPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

