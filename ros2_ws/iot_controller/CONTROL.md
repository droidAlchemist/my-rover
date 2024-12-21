## Subscribing / Publishing to IOT core topics 
## Sending twist message to control robot

colcon build
source ~/my-rover/ros2_ws/iot_controller/install/setup.bash
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
