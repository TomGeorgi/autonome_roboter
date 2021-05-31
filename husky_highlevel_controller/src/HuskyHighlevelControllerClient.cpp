#include "husky_highlevel_controller/HuskyHighlevelControllerClient.hpp"

namespace husky_highlevel_controller
{

    HuskyHighlevelControllerClient::HuskyHighlevelControllerClient(ros::NodeHandle &nh)
        : nh_(nh), ac_("husky_drive_action")
    {
        if (!readParameters())
        {
            ROS_ERROR("Coudl not read parameters!");
            ros::requestShutdown();
        }

        start_drive_service_ = nh_.advertiseService("start_drive", &HuskyHighlevelControllerClient::startDriveCB, this);
        stop_drive_service_ = nh_.advertiseService("stop_drive", &HuskyHighlevelControllerClient::stopDriveCB, this);

        ROS_INFO("Started husky_highlevel_contoller_client!");
    }

    HuskyHighlevelControllerClient::~HuskyHighlevelControllerClient()
    {
    }

    bool HuskyHighlevelControllerClient::readParameters()
    {
        if (!nh_.getParam("min_distance", min_distance_))
            return false;
        return true;
    }

    bool HuskyHighlevelControllerClient::startDriveCB(std_srvs::Empty::Request &req,
                                                      std_srvs::Empty::Response &res)
    {
        ROS_INFO("START DRIVE");
        husky_highlevel_controller::HuskyDriveGoal goal;
        goal.min_distance = min_distance_;
        ac_.sendGoal(goal, boost::bind(&HuskyHighlevelControllerClient::serverDoneCB, this, _1, _2),
                     boost::bind(&HuskyHighlevelControllerClient::serverActiveCB, this),
                     boost::bind(&HuskyHighlevelControllerClient::serverFeedbackCB, this, _1));
        return true;
    }

    bool HuskyHighlevelControllerClient::stopDriveCB(std_srvs::Empty::Request &req,
                                                     std_srvs::Empty::Response &res)
    {
        ac_.cancelAllGoals();
        return true;
    }

    void HuskyHighlevelControllerClient::serverActiveCB()
    {
        ROS_INFO("Server State went to active");
    }

    void HuskyHighlevelControllerClient::serverDoneCB(const actionlib::SimpleClientGoalState &state,
                                                      const husky_highlevel_controller::HuskyDriveResultConstPtr &result)
    {
        ROS_INFO("Server ended in state [%s]", state.toString().c_str());
    }

    void HuskyHighlevelControllerClient::serverFeedbackCB(const husky_highlevel_controller::HuskyDriveFeedbackConstPtr &feedback)
    {
        ROS_INFO("Server Feedback reached. Distance to pillar left: [%.2f]", feedback->distance);
    }

}