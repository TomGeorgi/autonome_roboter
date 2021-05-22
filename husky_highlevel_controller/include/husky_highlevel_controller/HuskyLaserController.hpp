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

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <husky_highlevel_controller/HuskyDriveAction.h>

#include "husky_highlevel_controller/Algorithm.hpp"

namespace husky_highlevel_controller
{
    /**
     * @brief HuskyLaserController Class is there to handle the Laser tasks.
     * 
     */
    class HuskyLaserController
    {
    private:
        // ROS node handle
        ros::NodeHandle nh_;

        // ROS Action client
        actionlib::SimpleActionClient<husky_highlevel_controller::HuskyDriveAction> ac_;

        // ROS subscriber of scan topic
        ros::Subscriber scan_subscriber_;

        // ROS publisher of a scan message
        ros::Publisher scan_publisher_;

        // ROS publisher of a marker message
        ros::Publisher vis_publisher_;

        // ROS service to start drive server
        ros::ServiceServer start_drive_service_;

        // ROS service tostop drive server
        ros::ServiceServer stop_drive_service_;

        // Name of the scan topic which should be subscribed
        std::string scan_topic_;

        // Name of the scan topic which should be published
        std::string scan_publisher_topic_;

        // Number of scan points in LaserScan message.
        double range_size_;

        // Algorithm object.
        Algorithm algorithm_;

        // Bool for drive or not
        bool drive_;

        // Angle
        double angle_;

        // Minimal Distance
        double min_val_;

        /**
         * @brief read parameters of from the ROS Parameter Server.
         *
         * @return true, all parameters were read succesfully.
         * @return false, a parameter couldn't be read.
         */
        bool readParameters();

        /**
         * @brief Service Callback to start drive.
         * 
         * @param req Empty request
         * @param res Empty response
         * @return true service finished successfully 
         * @return false service finished unsuccessfully
         */
        bool startDriveCB(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res);

        /**
         * @brief Service Callback to stop drive.
         * 
         * @param req Empty request
         * @param res Empty response
         * @return true service finished successfully 
         * @return false service finished unsuccessfully
         */
        bool stopDriveCB(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res);

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

        /**
         * @brief Server Active Callback Method.
         * 
         */
        void serverActiveCB();

        /**
         * @brief Callback method will be called when action server is in a done state.
         * 
         * @param state Action State
         * @param result Result
         */
        void serverDoneCB(const actionlib::SimpleClientGoalState &state,
                          const husky_highlevel_controller::HuskyDriveResultConstPtr &result);

        /**
         * @brief Drive Robot.
         * 
         */
        void drive();

    public:
        /**
         * @brief Construct a new Husky Laser Controller object
         * 
         * @param nh ROS NodeHandle object
         */
        HuskyLaserController(ros::NodeHandle &nh);

        /**
         * @brief Destroy the Husky Laser Controller object
         * 
         */
        virtual ~HuskyLaserController();
    };
} // namespace husky_highlevel_controller