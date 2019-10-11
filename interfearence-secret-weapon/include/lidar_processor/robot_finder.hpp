#ifndef __ROBOT_FINDER_HPP__
#define __ROBOT_FINDER_HPP__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>

class RobotFinder {
public:
    RobotFinder(std::string laser_topic,
                float arena_diameter,
                const float max_robot_side,
                const float min_robot_side,
                const float object_distance_threshold);
    ~RobotFinder();

private:
    void laserscan_cb(sensor_msgs::LaserScan::ConstPtr msg);

    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    // Odom of **ENEMY** robot
    ros::Publisher odom_pub_;

    const float arena_diameter_;
    const float max_robot_side_;
    const float min_robot_side_;
    const float object_distance_threshold_;

    bool first_run_;

    std::string tf_prefix;

    std::vector<nav_msgs::Odometry> previous_odoms_;
};

#endif // __ROBOT_FINDER_HPP__
