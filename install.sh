#!/bin/bash

if [ `whoami` != 'root' ]
	then
		echo "install.sh needs to be run with sudo!"
		exit
fi

echo "installing auro environment..."

# prerequisites:
sudo apt -y install python3-pip
python3 -m pip install osrf-pycommon

# install ros noetic
sudo apt -y install ros-noetic-desktop-full
sudo apt -y install ros-noetic-lms1xx
sudo apt -y install ros-noetic-velodyne-description
sudo apt -y install ros-noetic-twist-mus
sudo apt -y install ros-noetic-interactive-marker-twist-server
sudo apt -y install ros-noetic-robot-localization
sudo apt -y install ros-noetic-joy
sudo apt -y install ros-noetic-teleop-twist-joy

# install toolchain
sudo apt -y install python3-catkin-tools
sudo apt -y install python3-vcstool

# initialize catkin workspace
vcs import ../ < auro.repos
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# prepare husky environment
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro

echo "done"


