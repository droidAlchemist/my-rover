# Steps to setup UGV wave rover

## Install Ubuntu requirements 

apt-get update 
apt-get upgrade 

apt install python3-pip
apt-get install alsa-utils
apt install python3-colcon-argcomplete

apt install ros-foxy-cartographer-*
apt install ros-foxy-desktop-*
apt install ros-foxy-joint-state-publisher-*
apt install ros-foxy-nav2-*
apt install ros-foxy-rosbridge-*
apt install ros-foxy-rqt-*
apt install ros-foxy-rtabmap-*
apt install ros-foxy-usb-cam
apt install ros-foxy-depthai-*

#Simulation virtual machine installation
apt install gazebo
apt install ros-foxy-gazebo-*

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