/**
 * @file HuskyHighlevelControllerClient.hpp
 * @author Tom Georgi (Tom.Georgi@htwg-konstanz.de)
 * @author Christian Schmeisser (Christian.Schmeißer@htwg-konstanz.de)
 * @brief Contsins the HuskyHighlevelControllerClient
 * @version 0.1
 * @date 2021-06-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <husky_highlevel_controller/HuskyDriveAction.h>

namespace husky_highlevel_controller
{
    class HuskyHighlevelControllerClient
    {
    private:
        // ROS node handle
        ros::NodeHandle nh_;

        actionlib::SimpleActionClient<husky_highlevel_controller::HuskyDriveAction> ac_;

        // ROS service to start drive server
        ros::ServiceServer start_drive_service_;

        // ROS service to stop drive server
        ros::ServiceServer stop_drive_service_;

        // ROS Service to re-read parameters
        ros::ServiceServer read_parameter_server_service_;

        // Min allowed distance to pillar pose
        double min_distance_;

        bool readParameters();

        bool readParametersServiceCB(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res);

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
         * @brief Callback method will be called if a feedback from server is available.
         * 
         * @param feedback feedback.
         */
        void serverFeedbackCB(const husky_highlevel_controller::HuskyDriveFeedbackConstPtr &feedback);

    public:
        HuskyHighlevelControllerClient(ros::NodeHandle &nh);

        virtual ~HuskyHighlevelControllerClient();
    };
}