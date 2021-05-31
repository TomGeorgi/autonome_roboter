#include "husky_highlevel_controller/HuskyHighlevelControllerServer.hpp"

namespace husky_highlevel_controller
{
    HuskyHighlevelControllerServer::HuskyHighlevelControllerServer(ros::NodeHandle &nh)
        : nh_(nh), as_("husky_drive_action", boost::bind(&HuskyHighlevelControllerServer::executeCB, this, _1), false)
    {
        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters!");
            ros::requestShutdown();
        }

        cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);

        pillar_pose_subscriber_ = nh_.subscribe("pillar_pose", 1, &HuskyHighlevelControllerServer::pillarPoseCallback, this);

        read_parameter_server_service_ = nh_.advertiseService("read_parameters", &HuskyHighlevelControllerServer::readParametersServiceCB, this);

        as_.registerPreemptCallback(boost::bind(&HuskyHighlevelControllerServer::preemptCB, this));
        as_.start();
    }

    HuskyHighlevelControllerServer::~HuskyHighlevelControllerServer()
    {
        as_.shutdown();
    }

    bool HuskyHighlevelControllerServer::readParameters()
    {
        if (!nh_.getParam("cmd_vel_topic", cmd_vel_topic_))
            return false;
        if (!nh_.getParam("kp_param", kp_param_))
            return false;
        if (!nh_.getParam("linear_x", linear_x_))
            return false;
        if (!nh_.getParam("linear_y", linear_y_))
            return false;
        if (!nh_.getParam("angular_z", angular_z_))
            return false;
        return true;
    }

    bool HuskyHighlevelControllerServer::readParametersServiceCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        ROS_INFO("Reading parameters from param server");
        return readParameters();
    }

    void HuskyHighlevelControllerServer::pillarPoseCallback(const husky_highlevel_controller::PillarPose &pillar_pose)
    {
        pillar_pose_ = pillar_pose;
    }

    void HuskyHighlevelControllerServer::executeCB(const husky_highlevel_controller::HuskyDriveGoalConstPtr &goal)
    {
        min_distance_ = goal->min_distance;
        geometry_msgs::Twist new_cmd_vel;
        /*geometry_msgs::Pose pose;
        double angle = goal->pillar_pose.angle;
        double distance = goal->pillar_pose.distance;

        // Get (X, Y) Coordinates to publish marker
        double x = distance * cosf(angle);
        double y = distance * sinf(angle);

        std_msgs::ColorRGBA marker_color;
        visualization_msgs::Marker marker;
        marker_color.a = 1.0;
        marker_color.r = 1.0;
        marker_color.g = 0.0;
        marker_color.b = 1.0;

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = -1.5;
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, 0);

        // Transform marker to correct frame
        algorithm_.transformPose(pose, pose, goal->pillar_pose.src_frame_id, goal->pillar_pose.header.frame_id);

        marker = algorithm_.createMarker(x, y, goal->pillar_pose.src_frame_id, 1, marker_color);
        marker.pose = pose;

        vis_publisher_.publish(marker);
        */
        int feedback_size = 100;
        while (pillar_pose_.distance > min_distance_)
        {
            algorithm_.calculatePRatio(cmd_vel_, kp_param_, pillar_pose_.angle, new_cmd_vel);
            cmd_vel_ = new_cmd_vel;

            feedback_.distance = pillar_pose_.distance;
            if (feedback_size <= 0)
            {
                as_.publishFeedback(feedback_);
                feedback_size = 100;
            }
            feedback_size--;
            cmd_vel_publisher_.publish(cmd_vel_);
        }

        new_cmd_vel.angular.z = 0;
        new_cmd_vel.linear.x = 0;
        new_cmd_vel.linear.y = 0;
        cmd_vel_publisher_.publish(cmd_vel_);
        as_.setSucceeded(result_);
    }

    void HuskyHighlevelControllerServer::preemptCB()
    {
        ROS_WARN("Husky Drive Action: Preempted");

        geometry_msgs::Twist zero_vel;

        cmd_vel_publisher_.publish(zero_vel);

        as_.setPreempted();
    }
} // namespace husky_highlevel_controller
