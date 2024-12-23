## Subscribing / Publishing to IOT core topics 
## Sending twist message to control robot

## Python installations
sudo apt install python3-awscrt
python3 -m pip install awsiotsdk

## Build command
colcon build

## To start mock telemetry

source ~/my-rover/ros2_ws/iot_controller/install/setup.bash
ros2 run iot_controller mock_telemetry_pub
ros2 topic echo mock_telemetry

## To listen to IOT core

export IOT_CONFIG_FILE=~/my-rover/ros2_ws/iot_controller/iot_certs_and_config/iot_config.json
source ~/my-rover/ros2_ws/iot_controller/install/setup.bash

ros2 run iot_controller mqtt_telemetry_pub --ros-args --param path_for_config:=$IOT_CONFIG_FILE

ros2 run iot_controller mqtt_control_sub --ros-args --param path_for_config:=$IOT_CONFIG_FILE


## Test Data

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
