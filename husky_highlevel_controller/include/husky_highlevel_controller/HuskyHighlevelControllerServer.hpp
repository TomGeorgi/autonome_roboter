/**
 * @file HuskyDriveActionServer.hpp
 * @author Tom Georgi (Tom.Georgi@htwg-konstanz.de)
 * @author Christian Schmeisser (Christian.Schmeißer@htwg-konstanz.de)
 * @brief Contains the HuskyDriveActionServer Class Definition
 * @version 0.1
 * @date 2021-05-22
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <husky_highlevel_controller/HuskyDriveAction.h>
#include <husky_highlevel_controller/PillarPose.h>

#include "husky_highlevel_controller/Algorithm.hpp"

namespace husky_highlevel_controller
{
    class HuskyHighlevelControllerServer
    {
    private:
        // ROS node handle.
        ros::NodeHandle nh_;

        /**
         * @brief HuskyDrive Action Server
         * 
         * NOTE: NodeHandle instance must be created before this line. Otherwise strange error occurs.
         */
        actionlib::SimpleActionServer<husky_highlevel_controller::HuskyDriveAction> as_;

        // HuskyDrive feedback
        husky_highlevel_controller::HuskyDriveFeedback feedback_;

        // HuskyDriver result
        husky_highlevel_controller::HuskyDriveResult result_;

        // ROS publisher of a twist message
        ros::Publisher cmd_vel_publisher_;

        // ROS subscriber of custom message 'PillarPose'
        ros::Subscriber pillar_pose_subscriber_;

        // ROS Service to re-read parameters
        ros::ServiceServer read_parameter_server_service_;

        // Name of the twist topic which should be published
        std::string cmd_vel_topic_;

        geometry_msgs::Twist cmd_vel_;

        // Parameter for P-Controller.
        double kp_param_;

        // Parameter for start value of linear x velocity
        double linear_x_;

        // Parameter for start value of linear y velocity
        double linear_y_;

        // Parameter for start value of angular z velocity
        double angular_z_;

        husky_highlevel_controller::PillarPose pillar_pose_;

        double min_distance_;

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
         * @brief Service Callback for re-reading parameters
         *        from the ROS parameter server
         * 
         * @param req Empty request
         * @param res Empty response
         * @return true service finished successfully 
         * @return false service finished unsuccessfully
         */
        bool readParametersServiceCB(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res);

        /**
         * @brief Publish Command Velocity message
         * 
         * @param vel twist message for the command velocity
         */
        void publishCmdVel(const geometry_msgs::Twist &vel);

        /**
         * @brief pillar pose callback method
         * 
         * @param pillar_pose pillar pose message
         */
        void pillarPoseCallback(const husky_highlevel_controller::PillarPose &pillar_pose);

    public:
        /**
         * @brief Construct a new Husky Highlevel Controller Action Server object
         * 
         * @param nh 
         */
        HuskyHighlevelControllerServer(ros::NodeHandle &nh);

        /**
         * @brief Destroy the Husky Highlevel Controller Action Server object
         * 
         */
        virtual ~HuskyHighlevelControllerServer();

        /**
         * @brief Execute Callback on new Goal
         * 
         * @param goal goal
         */
        void executeCB(const husky_highlevel_controller::HuskyDriveGoalConstPtr &goal);

        /**
         * @brief Callback Method will be called on goal cancel.
         * 
         */
        void preemptCB();
    };
} // namespace husky_highlevel_controller
