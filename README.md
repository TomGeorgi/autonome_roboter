# Autonome Roboter

ROS1 Package for MSI-AS Lecture 'Autonome Roboter'

## Installation

First of all you need to install the ros tool `vcstool`:

```
sudo apt-get install python3-vcstool
```

Clone all dependend packages with the following command:

```
vcs import ../ < auro.repos
```

The next step is to install the following packages which are necessary for the *Husky*:

```
sudo apt install ros-noetic-lms1xx
sudo apt install ros-noetic-velodyne-description 
sudo apt install ros-noetic-twist-mux
sudo apt install ros-noetic-interactive-marker-twist-server
sudo apt install ros-noetic-robot-localization
sudo apt install ros-noetic-joy
sudo apt install ros-noetic-teleop-twist-joy
```


export the environment variabl **HUSKY_GAZEBO_DESCRIPTIOn** via:

```
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
```