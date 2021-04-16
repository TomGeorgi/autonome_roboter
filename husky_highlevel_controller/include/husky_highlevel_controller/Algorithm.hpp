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
};
}  // namespace husky_highlevel_controller