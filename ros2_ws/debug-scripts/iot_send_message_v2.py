from awsiot import mqtt5_client_builder
from awscrt import mqtt5, http
import json  
from concurrent.futures import Future

# Initialize path_for_config
path_for_config = '/home/joshua/my-rover/ros2_ws/src/iot_controller/iot_certs_and_config/iot_config.json'
future_stopped = Future()
future_connection_success = Future() 
message_topic = "ros2_telemetry_topic"

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

    # Callback for the lifecycle event Stopped
    def on_lifecycle_stopped(self, lifecycle_stopped_data: mqtt5.LifecycleStoppedData):
        print("Lifecycle Stopped")
        global future_stopped
        future_stopped.set_result(lifecycle_stopped_data)


    # Callback for the lifecycle event Connection Success
    def on_lifecycle_connection_success(self, lifecycle_connect_success_data: mqtt5.LifecycleConnectSuccessData):
        print("Lifecycle Connection Success")
        global future_connection_success
        future_connection_success.set_result(lifecycle_connect_success_data)


    # Callback for the lifecycle event Connection Failure
    def on_lifecycle_connection_failure(self, lifecycle_connection_failure: mqtt5.LifecycleConnectFailureData):
        print("Lifecycle Connection Failure")
        print("Connection failed with exception:{}".format(lifecycle_connection_failure.exception))
    
    def cleanup(self):
        # Unsubscribe
        print("Unsubscribing from topic '{}'".format(message_topic))
        unsubscribe_future = self.client.unsubscribe(unsubscribe_packet=mqtt5.UnsubscribePacket(
            topic_filters=[message_topic]))
        unsuback = unsubscribe_future.result(TIMEOUT)
        print("Unsubscribed with {}".format(unsuback.reason_codes))

        print("Stopping Client")
        self.client.stop()

        future_stopped.result(TIMEOUT)
        print("Client Stopped!")

    def connect_to_endpoint(self, cert_data):
        self.client = mqtt5_client_builder.mtls_from_path(
            endpoint=cert_data["endpoint"],
            port= cert_data["port"],
            cert_filepath= cert_data["certificatePath"],
            pri_key_filepath= cert_data["privateKeyPath"],
            ca_filepath= cert_data["rootCAPath"],
            client_id= cert_data["clientID"],
            http_proxy_options=None,
            on_lifecycle_stopped=self.on_lifecycle_stopped,
            on_lifecycle_connection_success=self.on_lifecycle_connection_success,
            on_lifecycle_connection_failure=self.on_lifecycle_connection_failure,
        )
        self.client.start()        
        lifecycle_connect_success_data = future_connection_success.result(TIMEOUT)
        connack_packet = lifecycle_connect_success_data.connack_packet
        negotiated_settings = lifecycle_connect_success_data.negotiated_settings
        # self.logger.info("Connected!")

def main():
    message_json = {
        "type": "voltage",
        "data": "10.01"
    }
    connection_helper = ConnectionHelper(path_for_config)
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