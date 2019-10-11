#include "lidar_processor/robot_finder.hpp"

RobotFinder::RobotFinder(std::string laser_topic, 
                         float arena_diameter,
                         float max_robot_side,
                         float min_robot_side,
                         float object_distance_threshold) 
        :arena_diameter_(arena_diameter), max_robot_side_(max_robot_side),
         min_robot_side_(min_robot_side), 
         object_distance_threshold_(object_distance_threshold) {
    laser_sub_ = nh_.subscribe(laser_topic, 
                               50, 
                               &RobotFinder::laserscan_cb, 
                               this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("enemy_vo", 50);

    nh_.getParam("tf_prefix", tf_prefix);
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
        if(msg->range_min < dist && dist < msg->range_max 
           && dist < arena_diameter_) {
            detections.push_back(Eigen::Vector3f(dist*cos(angle),
                                                 dist*sin(angle),
                                                 0));
        }
        angle += msg->angle_increment;
    }

    // If we didn't see anything, don't publish
    if(!detections.size()) return;

    // Split points into groups by location
    std::vector<std::vector<Eigen::Vector3f>> grouped_points;
    for(int i = 0; i < detections.size(); i++) {
        int j;
        for(j = i + 1; j < detections.size(); j++) {
            Eigen::Vector3f& prev = detections[j-1];
            Eigen::Vector3f& curr = detections[j];
            float point_distance = sqrt(
                pow(prev.x()-curr.x(), 2) + pow(prev.y()-curr.y(), 2));
            // Find the smallest distance of the previous 5 points
            for(int k = j-2; k > 0 && k > j-5; k--) {
                prev = detections[k];
                float dist = sqrt(
                    pow(prev.x()-curr.x(), 2) + pow(prev.y()-curr.y(), 2));
                if(dist < point_distance)
                    point_distance = dist;
            }
            // Stop looping if the points are separated enough to be 
            // different objects
            //
            // Due to the grating, we want to check up to 5 previous points
            if(point_distance > object_distance_threshold_)
                break;
        }
        // If i + 1 and j are different then store all the points from i to
        // j (not inclusive of j!!!)
        if(i + 1 != j) {
            grouped_points.push_back(std::vector<Eigen::Vector3f>());
            for(int k = i; k < j; k++)
                grouped_points.back().push_back(detections[k]);
        }
        i = j-1;
    }

    // Loop through all groups and find the AABB and size of the object
    // If it's greater than the maximum allowed or smaller than the minimum
    // then discard it
    std::vector<std::vector<Eigen::Vector3f>> remaining_groups;
    for(const std::vector<Eigen::Vector3f>& group : grouped_points) {
        // Loop through each point to find AABB
        float xmin, ymin, xmax, ymax;
        xmin = xmax = group[0].x();
        ymin = ymax = group[0].y();
        for(const Eigen::Vector3f& point : group) {
            if(point.x() < xmin) xmin = point.x();
            else if(point.x() > xmax) xmax = point.x();
            if(point.y() < ymin) ymin = point.y();
            else if(point.y() > ymax) ymax = point.y();
        }
        float x_side = xmax - xmin;
        float y_side = ymax - ymin;
        // x and y must both be less than the maximum, but
        // only one must be greater than the minimum
        if(x_side <= max_robot_side_ && y_side <= max_robot_side_ &&
           (min_robot_side_ <= y_side || min_robot_side_ <= x_side))
        {
            remaining_groups.push_back(group);
        }
    }

    // If there are no groups then return
    if(!remaining_groups.size()) {
        ROS_WARN_STREAM("[" << tf_prefix
                        << "] No groups of points found!!!");
        return;
    }

    // Now calculate the enemy position by averaging the position of the
    // remaining points for each group
    std::vector<Eigen::Vector3f> enemy_positions;

    for(const std::vector<Eigen::Vector3f>& group : remaining_groups) {
        enemy_positions.push_back(Eigen::Vector3f(0, 0, 0));
        for(const Eigen::Vector3f& point : group) 
            enemy_positions.back() += point;
        enemy_positions.back() /= group.size();
    }

    // Now select the most likely candidate for the enemy position.

    nav_msgs::Odometry enemy_odom;

    // If we have no previous position estimates, weigh based on how close
    // it is to an estimated starting distance
    if(!previous_odoms_.size()) {
        auto position_weigh = 
            [this](const Eigen::Vector3f& p) 
        {
            float mag = sqrt(p.x()*p.x() + p.y()*p.y()) - arena_diameter_/2;
            return fabs(mag);
        };

        float best_weight = position_weigh(enemy_positions[0]);
        Eigen::Vector3f& best_point = enemy_positions[0];

        for(const Eigen::Vector3f& point : enemy_positions) {
            if(position_weigh(point) < best_weight)
                best_point = point;
        }

        enemy_odom.pose.pose.position.x = best_point.x();
        enemy_odom.pose.pose.position.y = best_point.y();
        // Enemy hasn't started moving yet, so assume they are facing us
        // x, y as we set x to be forwards (i.e. if x is 1 and y is 0 the
        // result should be 0)
        // Negative as we want them to face us not them facing away from us
        // Subtracting pi/2 as we're offset by 90 degrees
        float yaw = -atan2(best_point.x(), best_point.y()) - 1.5707;
        enemy_odom.pose.pose.orientation.z = sin(yaw/2);
        enemy_odom.pose.pose.orientation.w = cos(yaw/2);
    }

    // If we have a single previous position estimate, weigh based on 
    // position relative to that
    else if(previous_odoms_.size() == 1) {
        
    }

    // If we have a previous position and velocity estimate, weigh based
    // on previous position + dt * velocity estimate
    else {

    }

    // Finally, publish the estimated position of the enemy robot
    enemy_odom.header.stamp = msg->header.stamp;
    if(tf_prefix != "") {
        enemy_odom.child_frame_id = tf_prefix + "/enemy";
        enemy_odom.header.frame_id = tf_prefix + "/base_link";
    }
    else {
        enemy_odom.child_frame_id = "enemy";
        enemy_odom.header.frame_id = "base_link";
    }

    // Covariance estimations - main thing here is that twist is less
    // reliable than pose
    enemy_odom.pose.covariance = {1e-3,    0,   0,   0,   0,   0,
                                     0, 1e-3,   0,   0,   0,   0,
                                     0,    0, 1e6,   0,   0,   0,
                                     0,    0,   0, 1e6,   0,   0,
                                     0,    0,   0,   0, 1e6,   0,
                                     0,    0,   0,   0,   0, 1e6};

    enemy_odom.twist.covariance = {1e-2,    0,   0,   0,   0,   0,
                                      0, 1e-2,   0,   0,   0,   0,
                                      0,    0, 1e6,   0,   0,   0,
                                      0,    0,   0, 1e6,   0,   0,
                                      0,    0,   0,   0, 1e6,   0,
                                      0,    0,   0,   0,   0, 1e6};
    odom_pub_.publish(enemy_odom);
}
