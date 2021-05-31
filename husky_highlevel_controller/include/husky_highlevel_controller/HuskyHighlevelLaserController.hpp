/**
 * @file HuskyLaserController.hpp
 * @author Tom Georgi (Tom.Georgi@htwg-konstanz.de)
 * @author Christian Schmeisser (Christian.Schmeißer@htwg-konstanz.de)
 * @brief Contains the HuskyLaserController Class Definition
 * @version 0.1
 * @date 2021-05-22
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <husky_highlevel_controller/HuskyDriveAction.h>

#include "husky_highlevel_controller/Algorithm.hpp"

namespace husky_highlevel_controller
{
    /**
     * @brief HuskyLaserController Class is there to handle the Laser tasks.
     * 
     */
    class HuskyHighlevelLaserController
    {
    private:
        // ROS node handle
        ros::NodeHandle nh_;

        // ROS subscriber of scan topic
        ros::Subscriber scan_subscriber_;

        // ROS publisher of a scan message
        ros::Publisher scan_publisher_;

        // ROS publisher of a marker message
        ros::Publisher vis_publisher_;

        // ROS publisher of a pillarpose message
        ros::Publisher pillar_pose_publisher_;

        // Name of the scan topic which should be subscribed
        std::string scan_topic_;

        // Name of the scan topic which should be published
        std::string scan_publisher_topic_;

        // Number of scan points in LaserScan message.
        double range_size_;

        // TF2 Buffer
        tf2_ros::Buffer tf_buffer_;

        // TF2 Transform Listener
        tf2_ros::TransformListener tf_listener_;

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
        void scanCallback(const sensor_msgs::LaserScan &msg);

        /**
		 * @brief publish a modified LaserScan Message.
		 *
		 * @param scan LaserScan message
		 * @param closest_index index of the minimal distance in LaserScan
		 * @param min_val mininmal distance in LaserScan
		 */
        void publishScans(const sensor_msgs::LaserScan &scan,
                          int closest_index, double min_val);

        void publishMarker(const double x, const double y,
                           const std::string frame_id,
                           const std::string frame_id_to_transform);

    public:
        /**
         * @brief Construct a new Husky Highlevel Laser Controller object
         * 
         * @param nh ROS NodeHandle object
         */
        HuskyHighlevelLaserController(ros::NodeHandle &nh);

        /**
         * @brief Destroy the Husky Highlevel Laser Controller object
         * 
         */
        virtual ~HuskyHighlevelLaserController();
    };
} // namespace husky_highlevel_controller