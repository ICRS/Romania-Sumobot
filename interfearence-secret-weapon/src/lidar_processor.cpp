#include <ros/ros.h>

#include "lidar_processor/robot_finder.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_processor");

    std::string laser_topic;
    float arena_diameter, max_robot_side, min_robot_side, 
          object_separation_dist, velocity_threshold;
    int odometry_memory;

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

    if(!nh_rel.getParam("max_robot_side", max_robot_side)) {
        ROS_FATAL("Failed to get param 'max_robot_side'");
        all_params_found = false;
    }

    if(!nh_rel.getParam("min_robot_side", min_robot_side)) {
        ROS_FATAL("Failed to get param 'min_robot_side'");
        all_params_found = false;
    }

    if(!nh_rel.getParam("object_separation_dist", object_separation_dist)) {
        ROS_FATAL("Failed to get param 'object_separation_dist'");
        all_params_found = false;
    }

    if(!nh_rel.getParam("velocity_threshold", velocity_threshold)) {
        ROS_FATAL("Failed to get param 'velocity_threshold'");
        all_params_found = false;
    }

    if(!nh_rel.getParam("odometry_memory", odometry_memory)) {
        ROS_FATAL("Failed to get param 'odometry_memory'");
        all_params_found = false;
    }

    if(!all_params_found) {
        ROS_FATAL("Not all parameters have been successfully loaded.");
        return -1;
    }

    RobotFinder finder(
        laser_topic, arena_diameter, max_robot_side, min_robot_side, 
        object_separation_dist, velocity_threshold, odometry_memory);

    finder.calibrate();

    ros::spin();
    return 0;
}
