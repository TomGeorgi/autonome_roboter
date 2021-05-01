/**
 * @file husky_highlevel_controller_node.cpp
 * @author Tom Georgi (Tom.Georgi@htwg-konstanz.de)
 * @author Christian Schmeisser (Christian.Schmeisser@htwg-konstanz.de)
 * @brief Main of husky highlevel controller ROS Node
 * @version 0.1
 * @date 2021-05-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>

#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");
  ros::NodeHandle nh("~");

  husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nh);

  ros::spin();
  return 0;
}