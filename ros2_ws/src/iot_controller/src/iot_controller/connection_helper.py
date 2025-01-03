from awsiot import mqtt5_client_builder
from awscrt import mqtt5, http
from concurrent.futures import Future
import json  
import uuid

SUBSCRIBE_IOT_CONTROL_TOPIC = "cmd_vel"
TIMEOUT = 100

class ConnectionHelper:
    def __init__(self, logger, path_for_config="", on_receive_func = None):
        self.path_for_config = path_for_config
        self.logger = logger
        self.future_stopped = Future()
        self.future_connection_success = Future() 

        self.on_receive_func = on_receive_func

        with open(path_for_config) as f:
          cert_data = json.load(f)

        self.logger.info("Config we are loading is :\n{}".format(cert_data))
        self.logger.info("Connecting directly to endpoint")
        self.connect_to_endpoint(cert_data)

     # Callback for the lifecycle event Stopped
    def on_lifecycle_stopped(self, lifecycle_stopped_data: mqtt5.LifecycleStoppedData):
        self.logger.info("Lifecycle Stopped")
        self.future_stopped.set_result(lifecycle_stopped_data)

    # Callback for the lifecycle event Connection Success
    def on_lifecycle_connection_success(self, lifecycle_connect_success_data: mqtt5.LifecycleConnectSuccessData):
        self.logger.info("Lifecycle Connection Success")
        self.future_connection_success.set_result(lifecycle_connect_success_data)

    # Callback for the lifecycle event Connection Failure
    def on_lifecycle_connection_failure(self, lifecycle_connection_failure: mqtt5.LifecycleConnectFailureData):
        self.logger.info("Lifecycle Connection Failure")
        self.logger.info("Connection failed with exception:{}".format(lifecycle_connection_failure.exception))
    
    def cleanup(self):
        # Unsubscribe
        self.logger.info("Unsubscribing from topic '{}'".format(SUBSCRIBE_IOT_CONTROL_TOPIC))
        unsubscribe_future = self.client.unsubscribe(unsubscribe_packet=mqtt5.UnsubscribePacket(
            topic_filters=[SUBSCRIBE_IOT_CONTROL_TOPIC]))
        unsuback = unsubscribe_future.result(TIMEOUT)
        self.logger.info("Unsubscribed with {}".format(unsuback.reason_codes))

        self.logger.info("Stopping Client")
        self.client.stop()

        self.future_stopped.result(TIMEOUT)
        self.logger.info("Client Stopped!")

    # Callback when any publish is received
    def on_publish_received(self, publish_packet_data):
        publish_packet = publish_packet_data.publish_packet
        assert isinstance(publish_packet, mqtt5.PublishPacket)
        topic = publish_packet.topic
        payload = publish_packet.payload
        self.logger.info("Received message from topic'{}':{}".format(topic, payload))
        if self.on_receive_func != None:
            self.on_receive_func(payload)

    def connect_to_endpoint(self, cert_data):
        clientID = cert_data["clientID"] + str(uuid.uuid4())
        self.client = mqtt5_client_builder.mtls_from_path(
            endpoint=cert_data["endpoint"],
            port= cert_data["port"],
            cert_filepath= cert_data["certificatePath"],
            pri_key_filepath= cert_data["privateKeyPath"],
            ca_filepath= cert_data["rootCAPath"],
            client_id= clientID,
            http_proxy_options=None,
            on_publish_received=self.on_publish_received,
            on_lifecycle_stopped=self.on_lifecycle_stopped,
            on_lifecycle_connection_success=self.on_lifecycle_connection_success,
            on_lifecycle_connection_failure=self.on_lifecycle_connection_failure,
        )
        self.logger.info("MQTT5 Client Created!")
        self.logger.info(f"Connecting to endpoint with Client ID '{clientID}'...")
        self.client.start()        
        lifecycle_connect_success_data = self.future_connection_success.result(TIMEOUT)
        connack_packet = lifecycle_connect_success_data.connack_packet
        negotiated_settings = lifecycle_connect_success_data.negotiated_settings
        self.logger.info(f"Connected to endpoint with Client ID:'{clientID}' with reason_code:{repr(connack_packet.reason_code)}")