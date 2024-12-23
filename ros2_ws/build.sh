cd /home/joshua/my-rover/ros2_ws
colcon build --packages-select iot_controller
colcon build --packages-select ugv_base_node ugv_interface
colcon build --packages-select ugv_bringup ugv_description ugv_tools
source install/setup.bash 

