#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>

#include "husky_highlevel_controller/HuskyLaserController.hpp"

namespace husky_highlevel_controller
{

    typedef actionlib::SimpleActionClient<husky_highlevel_controller::HuskyDriveAction> HuskyDriveClient;

    HuskyLaserController::HuskyLaserController(ros::NodeHandle &nh)
        : nh_(nh), ac_("husky_drive_action")
    {
        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters!");
            ros::requestShutdown();
        }

        drive_ = false;

        angle_ = -1.0;
        min_val_ = -1.0;

        scan_subscriber_ = nh_.subscribe(scan_topic_, 1, &HuskyLaserController::scanCallback, this);

        scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(scan_publisher_topic_, 10);

        vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

        start_drive_service_ = nh_.advertiseService("start_drive", &HuskyLaserController::startDriveCB, this);
        stop_drive_service_ = nh_.advertiseService("stop_drive", &HuskyLaserController::stopDriveCB, this);

        ROS_INFO("Started Scan Node!");
    }

    HuskyLaserController::~HuskyLaserController()
    {
    }

    bool HuskyLaserController::readParameters()
    {
        if (!nh_.getParam("scan_topic", scan_topic_))
            return false;
        if (!nh_.getParam("scan_publisher", scan_publisher_topic_))
            return false;
        if (!nh_.getParam("range_size", range_size_))
            return false;
        return true;
    }

    bool HuskyLaserController::startDriveCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        drive_ = true;
        return false;
    }

    bool HuskyLaserController::stopDriveCB(std_srvs::Empty::Request &req,
                                           std_srvs::Empty::Response &res)
    {
        ac_.cancelAllGoals();
        drive_ = false;
        return true;
    }

    void HuskyLaserController::scanCallback(const sensor_msgs::LaserScan &msg)
    {
        // Get minimal distance
        int closest_index;
        std::tie(closest_index, min_val_) = algorithm_.getMinimalDistance(msg);

        if (closest_index > 0)
        {
            geometry_msgs::Twist new_cmd_vel;
            angle_ = ((msg.angle_max - msg.angle_min) / 2) - closest_index * msg.angle_increment;

            // Get (X, Y) Coordinates to publish a marker
            double x = min_val_ * cosf(angle_);
            double y = min_val_ * sinf(angle_);

            std_msgs::ColorRGBA marker_color;
            visualization_msgs::Marker marker;
            marker_color.a = 1.0;
            marker_color.r = 0.0;
            marker_color.g = 1.0;
            marker_color.b = 0.0;

            marker = algorithm_.createMarker(x, y, "base_laser", 0, marker_color);
            vis_publisher_.publish(marker);
            if (drive_ && ac_.getState().isDone())
                drive();
        }

        publishScans(msg, closest_index, min_val_);
    }

    void HuskyLaserController::serverDoneCB(const actionlib::SimpleClientGoalState &state,
                                            const husky_highlevel_controller::HuskyDriveResultConstPtr &result)
    {
        ROS_INFO("Server ended in state [%s]", state.toString().c_str());
    }

    void HuskyLaserController::serverActiveCB()
    {
        ROS_INFO("Goal just went active");
    }

    void HuskyLaserController::drive()
    {
        husky_highlevel_controller::HuskyDriveGoal goal;
        goal.pillar_pose.header.frame_id = "odom";
        goal.pillar_pose.src_frame_id = "base_laser";
        goal.pillar_pose.distance = min_val_;
        goal.pillar_pose.angle = angle_;

        ac_.sendGoal(goal, boost::bind(&HuskyLaserController::serverDoneCB, this, _1, _2),
                     boost::bind(&HuskyLaserController::serverActiveCB, this),
                     HuskyDriveClient::SimpleFeedbackCallback());
    }

    void HuskyLaserController::publishScans(const sensor_msgs::LaserScan &scan, int closest_index, double min_val)
    {
        sensor_msgs::LaserScan new_scan;
        if (!algorithm_.createLaserScanAroundMD(scan, new_scan, closest_index, min_val, range_size_))
            return;

        // Publish new LaserScan object
        scan_publisher_.publish(new_scan);
    }
} // namespace husky_highlevel_controller
