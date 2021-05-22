#include <ros/ros.h>

#include <husky_highlevel_controller/HuskyLaserController.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_laser_controller");
    ros::NodeHandle nh("~");

    husky_highlevel_controller::HuskyLaserController husky_laser_controller(nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
    return 0;
}