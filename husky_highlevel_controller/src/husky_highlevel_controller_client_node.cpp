#include <ros/ros.h>

#include <husky_highlevel_controller/HuskyHighlevelControllerClient.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_highlevel_controller_client");
    ros::NodeHandle nh("~");

    husky_highlevel_controller::HuskyHighlevelControllerClient husky_highlevel_controller_client(nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
    return 0;
}