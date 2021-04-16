#include <ros/ros.h>

#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");
  ros::NodeHandle nh("~");

  husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nh);

  ros::spin();
  return 0;
}