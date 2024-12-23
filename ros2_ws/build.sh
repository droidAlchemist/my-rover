cd /home/joshua/my-rover/ros2_ws
export UGV_MODEL=ugv_rover
colcon build --packages-select iot_controller
# colcon build --packages-select ldlidar rf2o_laser_odometry robot_pose_publisher ugv_base_node ugv_interface
colcon build --packages-select costmap_converter_msgs costmap_converter emcl2 explore_lite openslam_gmapping slam_gmapping robot_pose_publisher teb_msgs teb_local_planner ugv_base_node ugv_interface
colcon build --packages-select ugv_bringup ugv_description ugv_nav ugv_slam ugv_tools
source install/setup.bash 

