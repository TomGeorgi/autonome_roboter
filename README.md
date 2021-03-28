# Autonome Roboter

ROS1 Package for MSI-AS Lecture 'Autonome Roboter'

## Prerequisit

First of all you need to install the ros tool `vcstool`:

```bash
sudo apt-get install python3-vcstool
sudo apt-get install python3-pip
```

Create your catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd src
```

# Installation

Clone this repository into the `src` directory.

Make sure that this repository is your current directory.
This must be the given for the next steps.

## Automatic installation

To install all dependencies run `./install.sh`.

## Manual installation

Clone all dependend packages with the following command into the `src` folder:

```bash
vcs import ../ < auro.repos
```

The next step is to install the following packages which are necessary for the *Husky*:

```bash
sudo apt install ros-noetic-lms1xx
sudo apt install ros-noetic-velodyne-description 
sudo apt install ros-noetic-twist-mux
sudo apt install ros-noetic-interactive-marker-twist-server
sudo apt install ros-noetic-robot-localization
sudo apt install ros-noetic-joy
sudo apt install ros-noetic-teleop-twist-joy
```

export the environment variabl **HUSKY_GAZEBO_DESCRIPTIOn** via:

```bash
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
```

## Setup your .bashrc

If you use both ros1 and ros2, you can create two aliases in your bashrc. These could look like the following:

```bash
# function with stuff for ros2
ros_two() {
    ...
}

# function with stuff for ros1 and this lecture
auro() {
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
}

# create aliases
alias ros2="ros_two"
alias auro="auro"
```

## Running Husky Simulation

Make sure that the file `setup.bash` is sourced either by:

```bash
source /opt/ros/noetic/setup.bash
```

or by calling the alias which was created with the code above:

```bash
auro
```

### Exercise 1

Build the catkin workspace:

```bash
cd to/your/catkin_ws
catkin build
# re-source your catkin workspace with:
auro
# or:
source ~/catkin_ws/devel/setup.bash
```

when the build is finished and the catkin_workspace is re-sourced the simulation can be started via:

```bash
roslaunch exercise_one exercise_one.launch
```

## Repository Structure

```bash
.
├── auro.repos                      # file with required repositories
├── autonome_roboter                # Metapackage contains:
│   ├── CMakeLists.txt              # CMakeList.txt (don't touch)
│   ├── images                      # Images for every exercise
│   │   ├── ...
│   └── package.xml                 # Package.xml (don't touch)
├── exercise_one                    # ROS-Package for exercise 1
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── exercise_one.launch
│   └── package.xml
├── general                         # ROS-Package with general stuff
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── general.launch          
│   └── package.xml
└── README.md
```
