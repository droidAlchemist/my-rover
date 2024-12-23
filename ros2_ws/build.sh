cd /home/joshua/my-rover/ros2_ws
export UGV_MODEL=ugv_rover
colcon build --packages-select iot_controller
colcon build --packages-select ldlidar robot_pose_publisher ugv_base_node ugv_interface
colcon build --packages-select ugv_bringup ugv_description ugv_tools
source install/setup.bash 

