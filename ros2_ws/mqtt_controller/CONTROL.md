## Subscribing to IOT core topic and sending twist message to control robot

colcon build
source ~/aws-iot-demo/ros2/install/setup.bash
ros2 run my_rover mqtt_control_sub --ros-args --param path_for_config:=$IOT_CONFIG_FILE


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
