#include "husky_highlevel_controller/Algorithm.hpp"

namespace husky_highlevel_controller
{
  Algorithm::Algorithm()
      : tf_listener_(tf_buffer_)
  {
  }

  Algorithm::~Algorithm() = default;

  std::tuple<int, double> Algorithm::getMinimalDistance(const sensor_msgs::LaserScan &scan)
  {
    int size = scan.ranges.size();
    int closest_index = -1;
    int min_index = 0;
    double min_val = 999; // values are between 0.2 and 30 meters for scanner
    for (int i = min_index; i < size; i++)
    {
      if (scan.ranges[i] < min_val)
      {
        min_val = scan.ranges[i];
        closest_index = i;
      }
    }

    return std::make_tuple(closest_index, min_val);
  }

  bool Algorithm::createLaserScanAroundMD(const sensor_msgs::LaserScan &old_scan, sensor_msgs::LaserScan &new_scan, int closest_index, double min_val, int range_size)
  {
    new_scan = old_scan;
    int size = new_scan.ranges.size();

    // Check if there is no object nearby
    if (closest_index == -1)
    {
      return false;
    }

    std::vector<float> new_ranges;
    int r_index, idx;

    for (int i = 0; i < range_size; i++)
    {
      idx = closest_index - (range_size * 0.5) + i;
      // r_index => If idx is out of range start from 0.
      r_index = (idx > size - 1) ? idx - (size - 1) : idx;
      new_ranges.push_back(old_scan.ranges[r_index]);
    }

    // Add new values to LaseScan object
    new_scan.ranges = new_ranges;
    new_scan.angle_min = old_scan.angle_min + (closest_index - range_size * 0.5) * new_scan.angle_increment;
    new_scan.angle_max = old_scan.angle_max + (closest_index + range_size * 0.5) * new_scan.angle_increment;

    return true;
  }

  void Algorithm::calculatePRatio(const geometry_msgs::Twist &old_cmd_vel, const double kp, double angle, geometry_msgs::Twist &new_cmd_vel)
  {
    new_cmd_vel.linear.x = kp * cosf(angle);
    new_cmd_vel.linear.y = kp * sinf(angle);
    // new_cmd_vel.linear.x = old_cmd_vel.linear.x;
    // new_cmd_vel.linear.y = old_cmd_vel.linear.y;
    new_cmd_vel.angular.z = kp * angle;
  }

  visualization_msgs::Marker Algorithm::createMarker(const double x, const double y,
                                                     const std::string frame_id,
                                                     const int marker_id,
                                                     const std_msgs::ColorRGBA marker_color)
  {
    visualization_msgs::Marker marker;
    geometry_msgs::Pose pose;
    geometry_msgs::TransformStamped transfrom_stamped;

    // Create Marker in 'frame_id' frame
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = -1.5;
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, 0);

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color = marker_color;

    return marker;
  }

  bool Algorithm::transformPose(const geometry_msgs::Pose src,
                                geometry_msgs::Pose dest,
                                const std::string src_frame_id,
                                const std::string dest_frame_id)
  {
    geometry_msgs::TransformStamped transform_stamped;

    try
    {
      transform_stamped = tf_buffer_.lookupTransform(dest_frame_id, src_frame_id, ros::Time(0));
      tf2::doTransform(src, dest, transform_stamped);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("%s", e.what());
      return false;
    }

    return true;
  }

} // namespace husky_highlevel_controller
