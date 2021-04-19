#include "husky_highlevel_controller/Algorithm.hpp"

namespace husky_highlevel_controller
{
Algorithm::Algorithm()
{
}

Algorithm::~Algorithm() = default;

std::tuple<int, double> Algorithm::getMinimalDistance(const sensor_msgs::LaserScan& scan)
{
  int size = scan.ranges.size();
  int closest_index = -1;
  int min_index = 0;
  double min_val = 999;  // values are between 0.2 and 30 meters for scanner
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

bool Algorithm::createLaserScanAroundMD(const sensor_msgs::LaserScan& old_scan, sensor_msgs::LaserScan& new_scan, int closest_index, double min_val, int range_size)
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

}  // namespace husky_highlevel_controller
