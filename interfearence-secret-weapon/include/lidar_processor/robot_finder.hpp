#ifndef __ROBOT_FINDER_HPP__
#define __ROBOT_FINDER_HPP__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class RobotFinder {
public:
    RobotFinder();
    ~RobotFinder();

private:
    void laserscan_cb(const sensor_msgs::LaserScan &msg);
};

#endif // __ROBOT_FINDER_HPP__
