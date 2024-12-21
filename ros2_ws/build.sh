cd /home/joshua/my-rover/ros2_ws
colcon build --packages-select iot_controller
colcon build --packages-select ugv_bringup ugv_tools
source install/setup.bash 

