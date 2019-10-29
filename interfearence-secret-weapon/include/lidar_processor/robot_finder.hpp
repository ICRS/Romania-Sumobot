#ifndef __ROBOT_FINDER_HPP__
#define __ROBOT_FINDER_HPP__

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>

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
    
    void calibrate();
private:
    void laserscan_cb(sensor_msgs::LaserScan::ConstPtr msg);
    void reset_cb(std_msgs::Bool::ConstPtr msg);
    void calibration_cb(sensor_msgs::LaserScan::ConstPtr msg);

    // Implementation of graham scan algorithm
    std::vector<Point> graham_scan(const std::vector<Point>& input);

    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Subscriber reset_sub_;
    // Odom of **ENEMY** robot
    ros::Publisher odom_pub_;
    ros::Publisher debug_point_cloud_pub_;

    const float arena_diameter_;
    const float max_robot_side_;
    const float min_robot_side_;
    const float object_distance_threshold_;
    const float velocity_threshold_;
    const int odometry_memory_;
    const std::string laser_topic_;

    bool first_run_;

    std::string tf_prefix;

    short calibration_data_size_;
    std::vector<std::vector<float>> calibration_data_;
    std::vector<int> ignore_scans_;

    std::vector<nav_msgs::Odometry> previous_odoms_;
};

#endif // __ROBOT_FINDER_HPP__
