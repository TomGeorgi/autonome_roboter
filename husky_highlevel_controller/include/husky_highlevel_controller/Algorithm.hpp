/**
 * @file Algorithm.hpp
 * @author Tom Georgi (Tom.Georgi@htwg-konstanz.de)
 * @author Christian Schmeisser (Christian.Schmeißer@htwg-konstanz.de)
 * @brief Contains algorithms
 * @version 0.1
 * @date 2021-04-16
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <sensor_msgs/LaserScan.h>

#include <tuple>

namespace husky_highlevel_controller
{
/**
 * @brief Class containing the algorithms of the package.
 *
 */
class Algorithm
{
public:
  /**
   * @brief Construct a new Algorithm object
   *
   */
  Algorithm();

  /**
   * @brief Destroy the Algorithm object
   *
   */
  virtual ~Algorithm();

  /**
   * @brief Get the Minimal Distance value and index from a LaserScan message.
   *
   * @param scan LaserScan Message
   * @return std::tuple<int, double> index of closest point, Minimal Distance
   */
  std::tuple<int, double> getMinimalDistance(const sensor_msgs::LaserScan& scan);

  /**
   * @brief Create a New Laser Scan message around the minimal distance.
   * 
   * @param old_scan Old Laser Scan Message
   * @param new_scan New Laser Scan Message
   * @param closest_index index of closest point in the laser scan message.
   * @param min_val value of the minimal distance in the laser scan message.
   * @param range_size range size around the minimal distance point.
   * @return bool, true for success false for fail
   */
  bool createLaserScanAroundMD(const sensor_msgs::LaserScan& old_scan, sensor_msgs::LaserScan& new_scan, int closest_index, double min_val, int range_size);
};
}  // namespace husky_highlevel_controller