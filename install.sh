#!/bin/bash

if [ `whoami` != 'root' ]
	then
		echo "install.sh needs to be run with sudo!"
		exit
fi

echo "installing auro environment..."

# prerequisites:
sudo apt -y install python3-pip

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
python3 -m pip install osrf-pycommon
sudo apt -y install python3-vcstool

# initialize catkin workspace
source /opt/ros/noetic/setup.bash
vcs import ../ < auro.repos
catkin build
source ../../devel/setup.bash

# prepare husky environment
# TODO: export not working yet! husky_gazebo not found!
#export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro

echo "done"

echo "Please add the required export and the sourcing of your catkin_ws manually to your .bashrc.\n"
echo "Read the manual (README.md) for the next steps."