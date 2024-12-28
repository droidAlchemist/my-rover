import json
import time
from awscrt import io
from awsiot import mqtt_connection_builder

class ConnectionHelper:
    def __init__(self, logger, path_for_config="", discover_endpoints=False):
        self.path_for_config = path_for_config
        self.discover_endpoints=discover_endpoints
        self.logger = logger

        with open(path_for_config) as f:
          cert_data = json.load(f)

        self.logger.info("Config we are loading is :\n{}".format(cert_data))
        self.logger.info("Connecting directly to endpoint")
        self.connect_to_endpoint(cert_data)

    def connect_to_endpoint(self, cert_data):
        self.mqtt_conn = mqtt_connection_builder.mtls_from_path(
            endpoint=cert_data["endpoint"],
            port= cert_data["port"],
            cert_filepath= cert_data["certificatePath"],
            pri_key_filepath= cert_data["privateKeyPath"],
            ca_filepath= cert_data["rootCAPath"],
            client_id= cert_data["clientID"],
            http_proxy_options=None,
        )
        connected_future = self.mqtt_conn.connect()
        connected_future.result()
        self.logger.info("Connected!")