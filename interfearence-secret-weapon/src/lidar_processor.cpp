#include <ros/ros.h>

#include "lidar_processor/robot_finder.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_processor");

    RobotFinder finder;
    return 0;
}
