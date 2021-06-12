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

**NOTE**:

If you receive the following error:

```bash
ros@desktop: catkin build 
Traceback (most recent call last):
  File "/usr/local/bin/catkin", line 5, in <module>
    from catkin_tools.commands.catkin import main
  File "/usr/local/lib/python3.8/dist-packages/catkin_tools/commands/catkin.py", line 28, in <module>
    from catkin_tools.common import is_tty
  File "/usr/local/lib/python3.8/dist-packages/catkin_tools/common.py", line 23, in <module>
    import trollius as asyncio
  File "/usr/local/lib/python3.8/dist-packages/trollius/init.py", line 21, in <module>
    from .base_events import *
  File "/usr/local/lib/python3.8/dist-packages/trollius/base_events.py", line 42, in <module>
    from . import tasks
  File "/usr/local/lib/python3.8/dist-packages/trollius/tasks.py", line 565
    def async(coro_or_future, loop=None):
        ^
SyntaxError: invalid syntax
```

Then type the following commands into your bash:

```bash
pip3 uninstall catkin_tools
sudo apt install python3-catkin-tools
```

You receiving this error due a bug in the catkin_tools package. See https://github.com/catkin/catkin_tools/issues/594.

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
sudo apt install ros-noetic-gmapping
sudo apt install ros-noetic-amcl
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-rviz-imu-plugin
sudo apt install ros-noetic-map-server
sudo apt install ros-noetic-dwa-local-planner
```

export the environment variabl **HUSKY_GAZEBO_DESCRIPTION** via:

```bash
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
```

For exercise 5 it's important to set the laser scan manually for gmapping via:

```bash
export HUSKY_LASER_LMS1XX_ENABLED=1
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
  export HUSKY_LASER_LMS1XX_ENABLED=1
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

## Setup your VS Code

See <https://github.com/RoboGnome/VS_Code_ROS>
