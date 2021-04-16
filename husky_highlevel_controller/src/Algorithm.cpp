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

}  // namespace husky_highlevel_controller
