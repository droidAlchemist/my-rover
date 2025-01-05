## Subscribing / Publishing to IOT core topics 
## Sending twist message to control robot

## Python installations
sudo apt install python3-awscrt
python3 -m pip install awsiotsdk

## Launch command
export IOT_CONFIG_FILE=~/my-rover/ros2_ws/src/iot_controller/iot_certs_and_config/iot_config.json
ros2 launch iot_controller iot_controller.launch.py 

## Build command (manual)
cd ros2_ws
colcon build --packages-select iot_controller

## To publish telemetry data

export IOT_CONFIG_FILE=~/my-rover/ros2_ws/src/iot_controller/iot_certs_and_config/iot_config.json
source ~/my-rover/ros2_ws/install/setup.bash
ros2 run iot_controller mqtt_publisher

eg. 
{
  "type": "voltage",
  "data": 11
}

## To listen to control

export IOT_CONFIG_FILE=~/my-rover/ros2_ws/src/iot_controller/iot_certs_and_config/iot_config.json
source ~/my-rover/ros2_ws/install/setup.bash

ros2 run iot_controller mqtt_listener

eg.

{
  "linear": {
    "x": 2.0,
    "y": 0.0,
    "z": 0.0
  },
  "angular": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  }
}
