from awsiot import mqtt5_client_builder
from awscrt import mqtt5, http
import json  
from concurrent.futures import Future

# Initialize path_for_config
path_for_config = '/home/joshua/my-rover/ros2_ws/src/iot_controller/iot_certs_and_config/iot_config.json'
future_stopped = Future()
future_connection_success = Future() 

TIMEOUT = 100

class ConnectionHelper:
    def __init__(self,  path_for_config="", discover_endpoints=False):
        self.path_for_config = path_for_config
        self.discover_endpoints=discover_endpoints
        # self.logger = logger

        with open(path_for_config) as f:
          cert_data = json.load(f)

        # self.logger.info("Config we are loading is :\n{}".format(cert_data))
        # self.logger.info("Connecting directly to endpoint")
        self.connect_to_endpoint(cert_data)

    def connect_to_endpoint(self, cert_data):
        self.mqtt_conn = mqtt5_client_builder.mtls_from_path(
            endpoint=cert_data["endpoint"],
            port= cert_data["port"],
            cert_filepath= cert_data["certificatePath"],
            pri_key_filepath= cert_data["privateKeyPath"],
            ca_filepath= cert_data["rootCAPath"],
            client_id= cert_data["clientID"],
            http_proxy_options=None,
        )
        connected_future = self.mqtt_conn.start()
        connected_future.result()
        # self.logger.info("Connected!")

def main():
    message_json = {
        "type": "voltage",
        "data": "10.01"
    }
    connection_helper = ConnectionHelper(path_for_config)
    message_topic = "ros2_telemetry_topic"
    message_string = message_json
    print("Publishing message to topic '{}': {}".format(message_topic, message_string))
    publish_future = connection_helper.mqtt_conn.publish(mqtt5.PublishPacket(
        topic=message_topic,
        payload=json.dumps(message_string),
        qos=mqtt5.QoS.AT_LEAST_ONCE
    ))

    publish_completion_data = publish_future.result(TIMEOUT)
    print("PubAck received with {}".format(repr(publish_completion_data.puback.reason_code)))

if __name__ == "__main__":
    main()