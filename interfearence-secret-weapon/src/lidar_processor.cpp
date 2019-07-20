#include <ros/ros.h>

#include "lidar_processor/robot_finder.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_processor");

    std::string laser_topic;
    float arena_diameter;
    float arena_safety_factor;

    ros::NodeHandle nh_rel("~");
    bool all_params_found = true;

    if(!nh_rel.getParam("laser_topic", laser_topic)) {
        ROS_FATAL("Failed to get param 'laser_topic'");
        all_params_found = false;
    }

    if(!nh_rel.getParam("arena_diameter", arena_diameter)) {
        ROS_FATAL("Failed to get param 'arena_diameter'");
        all_params_found = false;
    }

    if(!nh_rel.getParam("arena_safety_factor", arena_safety_factor)) {
        ROS_FATAL("Failed to get param 'arena_safety_factor'");
        all_params_found = false;
    }

    if(!all_params_found) {
        ROS_FATAL("Not all parameters have been successfully loaded.");
        return -1;
    }

    RobotFinder finder(laser_topic, arena_diameter, arena_safety_factor);
    ros::spin();
    return 0;
}
