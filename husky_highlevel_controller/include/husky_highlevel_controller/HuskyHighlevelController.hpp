/**
 * @file HuskyHighlevelController.hpp
 * @author Tom Georgi (Tom.Georgi@htwg-konstanz.de)
 * @author Christian Schmeisser (Christian.Schmeißer@htwg-konstanz.de)
 * @brief Contains the HuskyHighlevelController NOde
 * @version 0.1
 * @date 2021-04-16
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "husky_highlevel_controller/Algorithm.hpp"

namespace husky_highlevel_controller
{
/**
 * @brief HuskyHighlevelClass is there to handle the ROS interfacing for the node.
 *
 */
class HuskyHighlevelController
{
private:
	// ROS node handle.
	ros::NodeHandle nh_;

	// ROS topic subscriber
	ros::Subscriber subscriber_;

	// ROS topic publisher
	ros::Publisher publisher_;

	// Name of the scan topic which should be subscribed
	std::string scan_topic_;

	// Name of the scan topic which should be published
	std::string scan_publisher_;

	// Number of scan points in LaserScan message.
	double range_size_;

	// Algorithm object.
	Algorithm algorithm_;

	/**
	 * @brief read parameters of from the ROS Parameter Server.
	 *
	 * @return true, all parameters were read succesfully.
	 * @return false, a parameter couldn't be read.
	 */
	bool readParameters();

	/**
	 * @brief Callback Function for the scan_topic.
	 *
	 * @param msg LaseScan message.
	 */
	void scanCallback(const sensor_msgs::LaserScan& msg);

	/**
	 * @brief publish a modified LaserScan Message.
	 *
	 * @param scan LaserScan message
	 * @param closest_index index of the minimal distance in LaserScan
	 * @param min_val mininmal distance in LaserScan
	 */
	void publishScans(const sensor_msgs::LaserScan& scan, int closest_index, double min_val);

public:
	/**
	 * @brief Construct a new Husky Highlevel Controller object
	 *
	 * @param nh ROS NodeHandle object
	 */
	HuskyHighlevelController(ros::NodeHandle& nh);

	/**
	 * @brief Destroy the Husky Highlevel Controller object
	 *
	 */
	virtual ~HuskyHighlevelController();
};
}	 // namespace husky_highlevel_controller
