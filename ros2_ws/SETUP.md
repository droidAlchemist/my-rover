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