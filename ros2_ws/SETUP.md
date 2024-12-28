# Steps to setup UGV wave rover

## Install Ubuntu requirements 

sudo apt-get update 
sudo apt-get upgrade 

sudo apt install python3-pip
sudo apt-get install alsa-utils
sudo apt install python3-colcon-argcomplete

sudo apt install ros-$ROS_DISTRO-cartographer-* -y
sudo apt install ros-$ROS_DISTRO-desktop-* -y
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-* -y
sudo apt install ros-$ROS_DISTRO-nav2-* -y
sudo apt install ros-$ROS_DISTRO-rosbridge-* -y
sudo apt install ros-$ROS_DISTRO-rqt-* -y
sudo apt install ros-$ROS_DISTRO-rtabmap-* -y
sudo apt install ros-$ROS_DISTRO-usb-cam -y
sudo apt install ros-$ROS_DISTRO-depthai-* -y

#Simulation virtual machine installation
sudo apt install gazebo (failed - not found on ubuntu 24.04)
sudo apt install ros-$ROS_DISTRO-gazebo-*

## Install Python requirements 
cd /home/joshua/my-rover/ros2_ws
python3 -m pip install -r requirements.txt 
(optional --break-system-packages)

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



## Compile apriltag
cd /home/joshua/my-rover/ros2_ws
. build_apriltag.sh

## First Build
cd /home/joshua/my-rover/ros2_ws
. build_first.sh

## Daily Build

cd /home/joshua/my-rover/ros2_ws
. build_common.sh

# RPI 5 - Enable UART / Serial port ttyAMA0

sudo nano /boot/firmware/config.txt
[all]
enable_uart=1
dtoverlay=uart0

## AMA0 permission denied fix

sudo gpasswd --add ${USER} dialout

ls /dev/ttyA*