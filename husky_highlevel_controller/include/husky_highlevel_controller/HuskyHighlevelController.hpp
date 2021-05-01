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

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

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

    // ROS subscriber of scan topic
    ros::Subscriber scan_subscriber_;

    // ROS publisher of a scan message
    ros::Publisher scan_publisher_;

    // ROS publisher of a twist message
    ros::Publisher cmd_vel_publisher_;

    // ROS publisher of a marker message
    ros::Publisher vis_pub_;

    // ROS Service to re-read parameters
    ros::ServiceServer read_parameter_service_;

    // TF2 Buffer
    tf2_ros::Buffer tf_buffer_;

    // TF2 Transform Listener
    tf2_ros::TransformListener tf_listener_;

    // Name of the scan topic which should be subscribed
    std::string scan_topic_;

    // Name of the scan topic which should be published
    std::string scan_publisher_topic_;

    // Name of the twist topic which should be published
    std::string cmd_vel_topic_;

    geometry_msgs::Twist cmd_vel_;

    // Number of scan points in LaserScan message.
    double range_size_;

    // Parameter for P-Controller.
    double kp_param_;

    // Parameter for start value of linear x velocity
    double linear_x_;

    // Parameter for start value of linear y velocity
    double linear_y_;

    // Parameter for start value of angular z velocity
    double angular_z_;

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
     * @brief Publish Command Velocity message
     * 
     * @param vel twist message for the command velocity
     */
    void publishCmdVel(const geometry_msgs::Twist &vel);

    /**
     * @brief Publish Markers
     * 
     * @param x x-Position
     * @param y y-Position
     * @param frame_id frame_id for first marker
     * @param frame_id_to_transform frame_id for second marker
     */
    void publishMarker(const double x, const double y,
                       const std::string frame_id,
                       const std::string frame_id_to_transform);

  public:
    /**
		 * @brief Construct a new Husky Highlevel Controller object
		 *
		 * @param nh ROS NodeHandle object
		 */
    HuskyHighlevelController(ros::NodeHandle &nh);

    /**
     * @brief Destroy the Husky Highlevel Controller object
     *
     */
    virtual ~HuskyHighlevelController();
  };
} // namespace husky_highlevel_controller
