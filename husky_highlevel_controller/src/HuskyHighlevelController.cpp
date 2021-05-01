#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>

#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller
{
  HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle &nh)
      : nh_(nh), tf_listener_(tf_buffer_)
  {
    if (!readParameters())
    {
      ROS_ERROR("Could not read parameters!");
      ros::requestShutdown();
    }

    scan_subscriber_ = nh_.subscribe(scan_topic_, 1, &HuskyHighlevelController::scanCallback, this);

    scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(scan_publisher_topic_, 10);

    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);

    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    read_parameter_service_ = nh_.advertiseService("read_parameters", &HuskyHighlevelController::readParametersServiceCB, this);

    cmd_vel_.linear.x = linear_x_;
    cmd_vel_.linear.y = linear_y_;
    cmd_vel_.angular.z = angular_z_;

    ROS_INFO("Started scan node.");
  }

  HuskyHighlevelController::~HuskyHighlevelController()
  {
  }

  bool HuskyHighlevelController::readParameters()
  {
    if (!nh_.getParam("scan_topic", scan_topic_))
      return false;
    if (!nh_.getParam("cmd_vel_topic", cmd_vel_topic_))
      return false;
    if (!nh_.getParam("scan_publisher", scan_publisher_topic_))
      return false;
    if (!nh_.getParam("range_size", range_size_))
      return false;
    if (!nh_.getParam("kp_param", kp_param_))
      return false;
    if (!nh_.getParam("linear_x", linear_x_))
      return false;
    if (!nh_.getParam("linear_y", linear_y_))
      return false;
    if (!nh_.getParam("angular_z", angular_z_))
      return false;
    return true;
  }

  bool HuskyHighlevelController::readParametersServiceCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_INFO("Reading parameters from param server");
    return readParameters();
  }

  void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan &msg)
  {
    // Get Minimal Distance
    int closest_index;
    double min_val;
    std::tie(closest_index, min_val) = algorithm_.getMinimalDistance(msg);

    if (closest_index > 0)
    {
      // Get P Ratio
      geometry_msgs::Twist new_cmd_vel;
      double angle = ((msg.angle_max - msg.angle_min) / 2) - closest_index * msg.angle_increment;
      algorithm_.calculatePRatio(cmd_vel_, kp_param_, angle, new_cmd_vel);
      cmd_vel_ = new_cmd_vel;

      double x = min_val * cosf(angle);
      double y = min_val * sinf(angle);
      publishMarker(x, y, "base_laser", "odom");
    }

    cmd_vel_publisher_.publish(cmd_vel_);

    publishScans(msg, closest_index, min_val);
  }

  void HuskyHighlevelController::publishScans(const sensor_msgs::LaserScan &scan, int closest_index, double min_val)
  {
    sensor_msgs::LaserScan new_scan;
    if (!algorithm_.createLaserScanAroundMD(scan, new_scan, closest_index, min_val, range_size_))
      return;

    // Publish new LaserScan object
    scan_publisher_.publish(new_scan);
  }

  void HuskyHighlevelController::publishMarker(const double x, const double y,
                                               const std::string frame_id,
                                               const std::string frame_id_to_transform)
  {
    visualization_msgs::Marker marker;
    geometry_msgs::Pose pose;
    geometry_msgs::TransformStamped transform_stamped;

    // Create Message in base_laser
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = -1.5;
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, 0);

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // Publish marker
    vis_pub_.publish(marker);

    // Change color and if of new Marker
    marker.id = 1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    // Transform pose from base_laser to odom
    try
    {
      transform_stamped = tf_buffer_.lookupTransform(frame_id_to_transform, frame_id, ros::Time(0));
      marker.header.frame_id = frame_id_to_transform;
      tf2::doTransform(pose, pose, transform_stamped);
      marker.pose = pose;

      vis_pub_.publish(marker);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
  }
} // namespace husky_highlevel_controller
