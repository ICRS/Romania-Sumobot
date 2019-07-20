#include "lidar_processor/robot_finder.hpp"

RobotFinder::RobotFinder(std::string laser_topic, 
                         float arena_diameter,
                         float arena_sf) 
        :spinner_(1), arena_diameter_(arena_diameter), arena_sf_(arena_sf) {
    laser_sub_ = nh_.subscribe(laser_topic, 
                               50, 
                               &RobotFinder::laserscan_cb, 
                               this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("enemy_odom", 50);
}

RobotFinder::~RobotFinder() {
    // dtor
}

void RobotFinder::laserscan_cb(sensor_msgs::LaserScan::ConstPtr msg) {
    // Calculate 3d points relative to the laser scanner frame
    std::vector<Eigen::Vector3f> detections;
    float angle = msg->angle_min;
    float scans = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    for(int i = 0; i < scans; i++) {
        float dist = msg->ranges[i];
        if(msg->range_min < dist && dist < msg->range_max) {
            detections.push_back(Eigen::Vector3f(dist*cos(angle),
                                                dist*sin(angle),
                                                0));
        }
        angle += msg->angle_increment;
    }

    // Having calculated all points, now remove any that are outside the
    // arena, by getting the transform from the laser scanner to the arena
    // frame
    
    // TODO

    // Loop through remaining points and find the average position.
    Eigen::Vector3f average_pos(0, 0, 0);
    for(auto it = detections.begin(); it != detections.end(); it++)
        average_pos += *it;
    average_pos /= detections.size();

    double now = msg->header.stamp.sec 
               + msg->header.stamp.nsec / 1000000000;
    // Calculate delta pos to estimate velocity
    Eigen::Vector3f vel = (average_pos - prev_pos_) 
                        / (now - prev_time_);
    prev_pos_ = average_pos;
    prev_time_ = now;

    // Finally, publish the estimated position of the enemy robot
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = msg->header.frame_id;
    odom.child_frame_id = "enemy";
    odom.pose.pose.position.x = average_pos.x();
    odom.pose.pose.position.y = average_pos.y();
    odom.pose.pose.position.z = average_pos.z();

    // We don't estimate the orientation so an identity quaternion keeps
    // it's twist in the same frame as ours
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;

    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();

    // Once again we only measure linear pose
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;
    
    // Covariance estimations - main thing here is that twist is less
    // reliable than pose
    odom.pose.covariance = {1e-3,    0,   0,   0,   0,   0,
                               0, 1e-3,   0,   0,   0,   0,
                               0,    0, 1e6,   0,   0,   0,
                               0,    0,   0, 1e6,   0,   0,
                               0,    0,   0,   0, 1e6,   0,
                               0,    0,   0,   0,   0, 1e6};

    odom.twist.covariance = {1e-2,    0,   0,   0,   0,   0,
                                0, 1e-2,   0,   0,   0,   0,
                                0,    0, 1e6,   0,   0,   0,
                                0,    0,   0, 1e6,   0,   0,
                                0,    0,   0,   0, 1e6,   0,
                                0,    0,   0,   0,   0, 1e6};
    odom_pub_.publish(odom);
}
