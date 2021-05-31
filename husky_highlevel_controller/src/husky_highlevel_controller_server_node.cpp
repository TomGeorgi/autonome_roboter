#include <ros/ros.h>

#include <husky_highlevel_controller/HuskyHighlevelControllerServer.hpp>
#include <husky_highlevel_controller/HuskyHighlevelLaserController.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_highlevel_controller_server");
    ros::NodeHandle nh("~");

    husky_highlevel_controller::HuskyHighlevelControllerServer husky_highlevel_controller_server(nh);
    husky_highlevel_controller::HuskyHighlevelLaserController husky_highlevel_laser_controller(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();
    return 0;
}