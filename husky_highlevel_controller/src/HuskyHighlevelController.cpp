#include <string>

#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller
{
HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nh) : nh_(nh)
{
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters!");
    ros::requestShutdown();
  }

  subscriber_ = nh_.subscribe(scan_topic_, 1, &HuskyHighlevelController::scanCallback, this);

  publisher_ = nh_.advertise<sensor_msgs::LaserScan>(scan_publisher_, 10);

  ROS_INFO("Started scan node.");
}

HuskyHighlevelController::~HuskyHighlevelController()
{
}

bool HuskyHighlevelController::readParameters()
{
  if (!nh_.getParam("scan_topic", scan_topic_))
    return false;
  if (!nh_.getParam("scan_publisher", scan_publisher_))
    return false;
  if (!nh_.getParam("range_size", range_size_))
    return false;
  return true;
}

void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg)
{
  // Get Minimal Distance
  int closest_index;
  double min_val;
  std::tie(closest_index, min_val) = algorithm_.getMinimalDistance(msg);

  // Print out the Minimal Distance
  // ROS_INFO("Minimal Distance: %f m\nIndex: %d", min_val, closest_index);

  publishScans(msg, closest_index, min_val);
}

void HuskyHighlevelController::publishScans(const sensor_msgs::LaserScan& scan, int closest_index, double min_val)
{
  sensor_msgs::LaserScan new_scan;
  if(!algorithm_.createLaserScanAroundMD(scan, new_scan, closest_index, min_val, range_size_))
    return;

  // Publish new LaserScan object
  publisher_.publish(new_scan);
}
}  // namespace husky_highlevel_controller
