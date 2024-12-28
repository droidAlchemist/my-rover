# Steps to setup UGV wave rover

## Install Ubuntu requirements 

sudo apt-get update 
sudo apt-get upgrade 

sudo apt install python3-pip
sudo apt-get install alsa-utils
sudo apt install python3-colcon-argcomplete

sudo apt install ros-$ROS_DISTRO-cartographer-*
sudo apt install ros-$ROS_DISTRO-desktop-*
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-*
sudo apt install ros-$ROS_DISTRO-nav2-*
sudo apt install ros-$ROS_DISTRO-rosbridge-*
sudo apt install ros-$ROS_DISTRO-rqt-*
sudo apt install ros-$ROS_DISTRO-rtabmap-*
sudo apt install ros-$ROS_DISTRO-usb-cam
sudo apt install ros-$ROS_DISTRO-depthai-*

#Simulation virtual machine installation
sudo apt install gazebo
sudo apt install ros-$ROS_DISTRO-gazebo-*

## Install Python requirements 
cd /home/joshua/my-rover/ros2_ws
python3 -m pip install -r requirements.txt 
(optional --break-system-packages)


## First Build
cd /home/joshua/my-rover/ros2_ws
. build_first.sh

## Daily Build

cd /home/joshua/my-rover/ros2_ws
. build_common.sh

## Compile apriltag
cd /home/joshua/my-rover/ros2_ws
. build_apriltag.sh

# RPI 5 - Enable UART / Serial port ttyAMA0

sudo nano /boot/firmware/config.txt
[all]
enable_uart=1
dtoverlay=uart0

## AMA0 permission denied fix

sudo gpasswd --add ${USER} dialout

ls /dev/ttyA*