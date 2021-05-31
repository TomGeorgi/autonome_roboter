#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <husky_highlevel_controller/PillarPose.h>
#include <tf2/utils.h>

#include "husky_highlevel_controller/HuskyHighlevelLaserController.hpp"

namespace husky_highlevel_controller
{

    HuskyHighlevelLaserController::HuskyHighlevelLaserController(ros::NodeHandle &nh)
        : nh_(nh), tf_listener_(tf_buffer_)
    {
        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters!");
            ros::requestShutdown();
        }

        scan_subscriber_ = nh_.subscribe(scan_topic_, 1, &HuskyHighlevelLaserController::scanCallback, this);

        scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(scan_publisher_topic_, 10);

        vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

        pillar_pose_publisher_ = nh_.advertise<husky_highlevel_controller::PillarPose>("pillar_pose", 10);

        ROS_INFO("Started Scan Node!");
    }

    HuskyHighlevelLaserController::~HuskyHighlevelLaserController()
    {
    }

    bool HuskyHighlevelLaserController::readParameters()
    {
        if (!nh_.getParam("scan_topic", scan_topic_))
            return false;
        if (!nh_.getParam("scan_publisher", scan_publisher_topic_))
            return false;
        if (!nh_.getParam("range_size", range_size_))
            return false;
        return true;
    }

    void HuskyHighlevelLaserController::scanCallback(const sensor_msgs::LaserScan &msg)
    {
        // Get minimal distance
        int closest_index;
        double min_val, angle;
        std::tie(closest_index, min_val) = algorithm_.getMinimalDistance(msg);

        if (closest_index >= 0)
        {
            angle = ((msg.angle_max - msg.angle_min) / 2) - closest_index * msg.angle_increment;

            // Create Pillar Pose message
            PillarPose pillar_pose_message;
            pillar_pose_message.header.frame_id = "base_laser";
            pillar_pose_message.angle = angle;
            pillar_pose_message.distance = min_val;
            pillar_pose_publisher_.publish(pillar_pose_message);

            // Publish Markers
            // Get (X, Y) Coordinates to publish a marker
            double x = min_val * cosf(angle);
            double y = min_val * sinf(angle);

            publishMarker(x, y, "base_laser", "odom");
        }

        publishScans(msg, closest_index, min_val);
    }

    void HuskyHighlevelLaserController::publishScans(const sensor_msgs::LaserScan &scan, int closest_index, double min_val)
    {
        sensor_msgs::LaserScan new_scan;
        if (!algorithm_.createLaserScanAroundMD(scan, new_scan, closest_index, min_val, range_size_))
            return;

        // Publish new LaserScan object
        scan_publisher_.publish(new_scan);
    }

    void HuskyHighlevelLaserController::publishMarker(const double x, const double y,
                                                      const std::string frame_id,
                                                      const std::string frame_id_to_transform)
    {
        visualization_msgs::Marker marker;
        geometry_msgs::Pose pose;
        geometry_msgs::TransformStamped transform_stamped;

        // Create Message in base_laser
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = -1.5;
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, 0);

        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // Publish marker
        vis_publisher_.publish(marker);

        // Change color and if of new Marker
        marker.id = 1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        // Transform pose from base_laser to odom
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(frame_id_to_transform, frame_id, ros::Time(0));
            marker.header.frame_id = frame_id_to_transform;
            tf2::doTransform(pose, pose, transform_stamped);
            marker.pose = pose;

            vis_publisher_.publish(marker);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
    }
} // namespace husky_highlevel_controller
