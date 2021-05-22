#include <ros/ros.h>

#include <husky_highlevel_controller/HuskyDriveActionServer.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_drive_action_server");
    ros::NodeHandle nh("~");

    husky_highlevel_controller::HuskyDriveActionServer husky_drive_action_server(nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
    return 0;
}