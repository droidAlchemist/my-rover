# Ubuntu 24.04 fixes and ROS Jazzy fixes
# #######################################

- Some packages were updated from source git locations.
- Some packages were replaced by similar packages.
- cpp errors were solved by including files
- few cpp errors solved by converting .h to .hpp

## emcl2_ros2 Ubuntu 24.04 fixes
cd ros2_ws/src/ugv_else/em
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2/pull/59/files
Add to below files:
include/emcl2/Pose.h: line 9
include/emcl2/Scan.h: line 9
#include <cstdint>

src/emcl2_node.cpp: line 129
std::shared_ptr<LikelihoodFieldMap> map = initMap();
std::shared_ptr<OdomModel> om = initOdometry();

## ldlidar build error
https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2/pull/24/files 
ldlidar_driver/src/logger/log_module.cpp
line 28 add: #include <pthread.h>

## costmap converter error
http://wiki.ros.org/costmap_converter
downloaded and replaced existing package

## explore_lite ubuntu 24 fixes
https://github.com/hrnr/m-explore
did not work

https://github.com/robo-friends/m-explore-ros2/tree/main/explore
download and copied explore folder as explore_lite

## ros april tag ubuntu 24 fixes
apriltag_ros/include/AprilTagNode.hpp:40:10: fatal error: cv_bridge/cv_bridge.h
change to cv_bridge/cv_bridge.hpp

new error
ugv_else/apriltag_ros/apriltag_ros/include/AprilTagNode.hpp
image_geometry/pinhole_camera_model.h

https://github.com/christianrauch/apriltag_msgs 
Try switching to new packages by Christian rauch -> apriltag_ros, apriltag_msgs

## slam_gmapping ubuntu 24 fixes
ugv_else/gmapping/slam_gmapping/include/slam_gmapping/slam_gmapping.h  line 38
Move it to tf2_geometry_msgs.hpp instead.