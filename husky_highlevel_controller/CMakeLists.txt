cmake_minimum_required(VERSION 3.0.2)
project(husky_highlevel_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_definitions(--std=c++11)


## Find catkin macros and libraries
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib_msgs
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf
  tf2
  tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PillarPose.msg
)

add_action_files(
  FILES
  HuskyDrive.action
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS 
    include
#  LIBRARIES husky_highlevel_controller
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    sensor_msgs
    geometry_msgs
    std_srvs
    visualization_msgs
    tf
    tf2
    tf2_ros
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/husky_highlevel_controller.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(husky_highlevel_controller_client
  src/husky_highlevel_controller_client_node.cpp
  src/HuskyHighlevelControllerClient.cpp
)

add_executable(husky_highlevel_controller_server
  src/husky_highlevel_controller_server_node.cpp
  src/HuskyHighlevelControllerServer.cpp
  src/HuskyHighlevelLaserController.cpp
  src/Algorithm.cpp
)

target_compile_features(husky_highlevel_controller_client INTERFACE cxx_std_11)
target_compile_features(husky_highlevel_controller_server INTERFACE cxx_std_11)

add_dependencies(husky_highlevel_controller_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(husky_highlevel_controller_server ${PROJECT_NAME}_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(husky_highlevel_controller_client
  ${catkin_LIBRARIES}
)

target_link_libraries(husky_highlevel_controller_server
  ${catkin_LIBRARIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_husky_highlevel_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
