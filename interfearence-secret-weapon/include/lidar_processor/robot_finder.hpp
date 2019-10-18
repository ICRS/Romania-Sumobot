#ifndef __ROBOT_FINDER_HPP__
#define __ROBOT_FINDER_HPP__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>

#include <stack>
#include <algorithm>
#include <vector>

#include "dbscan/dbscan.hpp"

class RobotFinder {
public:
    RobotFinder(std::string laser_topic,
                const float arena_diameter,
                const float max_robot_side,
                const float min_robot_side,
                const float object_distance_threshold,
                const float velocity_threshold,
                const int odometry_memory);
    ~RobotFinder();

private:
    void laserscan_cb(sensor_msgs::LaserScan::ConstPtr msg);

    // Implementation of graham scan algorithm
    std::vector<Point> graham_scan(const std::vector<Point>& input);

    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    // Odom of **ENEMY** robot
    ros::Publisher odom_pub_;

    const float arena_diameter_;
    const float max_robot_side_;
    const float min_robot_side_;
    const float object_distance_threshold_;
    const float velocity_threshold_;
    const int odometry_memory_;

    bool first_run_;

    std::string tf_prefix;

    std::vector<nav_msgs::Odometry> previous_odoms_;
};

#endif // __ROBOT_FINDER_HPP__
