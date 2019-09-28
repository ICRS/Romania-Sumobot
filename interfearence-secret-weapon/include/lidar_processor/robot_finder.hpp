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
                float arena_safety_factor);
    ~RobotFinder();

private:
    void laserscan_cb(sensor_msgs::LaserScan::ConstPtr msg);

    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    // Odom of **ENEMY** robot
    ros::Publisher odom_pub_;

    Eigen::Vector3f prev_pos_;
    double prev_time_;

    const float arena_diameter_;
    const float arena_sf_;

    bool first_run_;

    std::string tf_prefix;
};

#endif // __ROBOT_FINDER_HPP__
