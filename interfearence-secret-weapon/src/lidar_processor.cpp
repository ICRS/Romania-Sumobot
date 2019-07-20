#include <ros/ros.h>

#include "lidar_processor/robot_finder.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_processor");

    std::string laser_topic;
    float arena_diameter;
    float arena_safety_factor;

    RobotFinder finder(laser_topic, arena_diameter, arena_safety_factor);
    ros::spin();
    return 0;
}
