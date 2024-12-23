# Steps to setup UGV wave rover

python3 -m pip install -r requirements.txt --break-system-packages

colcon build --packages-select ugv_bringup ugv_tools
source install/setup.bash

## Drive the car (can control the pan/tilt and LED lights)

ros2 run ugv_bringup ugv_driver