#include "lidar_processor/robot_finder.hpp"

// 10cm a second is pretty slow...
const float VEL_THRESH = 0.1;
const int ODOMETRY_MEMORY = 5;

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
    std::vector<Point> detections;
    float angle = msg->angle_min;
    float scans = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    for(int i = 0; i < scans; i++) {
        float dist = msg->ranges[i];
        if(msg->range_min < dist && dist < msg->range_max 
           && dist < arena_diameter_) {
            detections.push_back(Point(dist*cos(angle),
                                       dist*sin(angle),
                                       0));
        }
        angle += msg->angle_increment;
    }

    // If we didn't see anything, don't publish
    if(!detections.size()) return;

    // Split points into groups by location
    std::vector<std::vector<Point>> grouped_points;

    // Minimum points together for a group to be found
    const int MINIMUM_POINTS = 5;
    // EPSILON is in m^2
    const float EPSILON = 
        object_distance_threshold_*object_distance_threshold_;

    // Test dbscan
    DBSCAN ds(MINIMUM_POINTS, EPSILON, detections);
    ds.run();

    for(int i = 0; i < ds.getTotalPointSize(); i++) {
        Point p = ds.m_points[i];
        if(p.clusterID == UNCLASSIFIED)
            continue;
        while(p.clusterID > grouped_points.size())
            grouped_points.push_back({});
        grouped_points[p.clusterID - 1].push_back(p);
    }

    // Loop through all groups and find the AABB and size of the object
    // If it's greater than the maximum allowed or smaller than the minimum
    // then discard it
    std::vector<std::vector<Point>> remaining_groups;
    for(const std::vector<Point>& group : grouped_points) {
        // Loop through each point to find AABB
        float xmin, ymin, xmax, ymax;
        xmin = xmax = group[0].x;
        ymin = ymax = group[0].y;
        for(const Point& point : group) {
            if(point.x < xmin) xmin = point.x;
            else if(point.x > xmax) xmax = point.x;
            if(point.y < ymin) ymin = point.y;
            else if(point.y > ymax) ymax = point.y;
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
    std::vector<Point> enemy_positions;

    for(const std::vector<Point>& group : remaining_groups) {
        enemy_positions.push_back(Point(0, 0, 0));
        for(const Point& point : group) 
            enemy_positions.back() += point;
        enemy_positions.back() /= group.size();
    }

    // Now select the most likely candidate for the enemy position.

    nav_msgs::Odometry enemy_odom;

    // If we have no previous position estimates, weigh based on how close
    // it is to an estimated starting distance
    if(!previous_odoms_.size()) {
        auto position_weigh = 
            [this](const Point& p) 
        {
            float mag = sqrt(p.x*p.x + p.y*p.y) - arena_diameter_/2;
            return fabs(mag);
        };

        float best_weight = position_weigh(enemy_positions[0]);
        Point& best_point = enemy_positions[0];

        for(const Point& point : enemy_positions) {
            if(position_weigh(point) < best_weight)
                best_point = point;
        }

        enemy_odom.pose.pose.position.x = best_point.x;
        enemy_odom.pose.pose.position.y = best_point.y;
        // Enemy hasn't started moving yet, so assume they are facing us
        // x, y as we set x to be forwards (i.e. if x is 1 and y is 0 the
        // result should be 0)
        // Negative as we want them to face us not them facing away from us
        // Subtracting pi/2 as we're offset by 90 degrees
        float yaw = -atan2(best_point.x, best_point.y) - 1.5707;
        enemy_odom.pose.pose.orientation.z = sin(yaw/2);
        enemy_odom.pose.pose.orientation.w = cos(yaw/2);
    }

    // If we have a single previous position estimate, weigh based on 
    // position relative to that
    else if(previous_odoms_.size() == 1) {
        auto prev = previous_odoms_.back().pose.pose.position;
        auto ros_dt = msg->header.stamp 
                    - previous_odoms_.back().header.stamp;
        float dt = ros_dt.sec + ros_dt.nsec / 1000000000.f;
        if(dt == 0) 
            return;
        auto position_weigh = [prev](const Point& p) {
            return pow(p.x - prev.x, 2) + pow(p.y - prev.y, 2);
        };

        float best_weight = position_weigh(enemy_positions[0]);
        Point& best_point = enemy_positions[0];

        for(const Point& point : enemy_positions) {
            if(position_weigh(point) < best_weight)
                best_point = point;
        }

        enemy_odom.pose.pose.position.x = best_point.x;
        enemy_odom.pose.pose.position.y = best_point.y;

        // We can now estimate the velocity as the delta position
        float dx = best_point.x - prev.x;
        float dy = best_point.y - prev.y;
        float velocity = sqrt(pow(dx / dt, 2) + pow(dy / dt, 2));
        ROS_INFO_STREAM("velocity: " << velocity);
        enemy_odom.twist.twist.linear.x = velocity;
//        enemy_odom.twist.twist.linear.x = (best_point.x - prev.x) / dt;
//        enemy_odom.twist.twist.linear.y = (best_point.y - prev.y) / dt;

        // If the velocity is greater than a threshold then we calculate a
        // new orientation
        if(velocity > VEL_THRESH) {
            // Take the change in position and calculate the orientation
            // assuming that the enemy moves in a straight line
            float yaw = atan2(dy, dx);
            enemy_odom.pose.pose.orientation.z = sin(yaw/2);
            enemy_odom.pose.pose.orientation.w = cos(yaw/2);
        }
        else {
            // We don't have enough data to estimate the enemy pose so 
            // assume yaw is unchanged.
            enemy_odom.pose.pose.orientation = 
                previous_odoms_.back().pose.pose.orientation;
        }
    }

    // If we have a previous position and velocity estimate, weigh based
    // on previous position + dt * velocity estimate
    else {
        auto ros_dt 
            = msg->header.stamp - previous_odoms_.back().header.stamp;
        float dt = ros_dt.sec + ros_dt.nsec / 1000000000.f;
        if(dt == 0)
            return;
        auto prev = previous_odoms_.back().pose.pose;
        auto vel = previous_odoms_.back().twist.twist.linear;
        auto position_weigh = [ros_dt, dt, prev, vel](const Point& p) {
            float vx, vy, yaw;
            yaw = 2*acos(prev.orientation.w);
            vx = vel.x * cos(yaw);
            vy = vel.y * sin(yaw);
            float mag = pow(p.x - (prev.position.x + vx * dt), 2) 
                      + pow(p.y - (prev.position.y + vy * dt), 2);
            return mag;
        };

        float best_weight = position_weigh(enemy_positions[0]);
        Point& best_point = enemy_positions[0];

        for(const Point& point : enemy_positions) {
            if(position_weigh(point) < best_weight)
                best_point = point;
        }

        enemy_odom.pose.pose.position.x = best_point.x;
        enemy_odom.pose.pose.position.y = best_point.y;

        // Calculate velocity by looking at the previous motion of the enemy
        // robot
        float dx = best_point.x - prev.position.x;
        float dy = best_point.y - prev.position.y;
        float velocity = sqrt(pow(dx / dt, 2) + pow(dy / dt, 2));

        float multiplier = 1;
        // Moving average of previous velocity estimates
        for(int i = 0; i < previous_odoms_.size(); i++) {
            float exp = 1.0 / pow(2, previous_odoms_.size() - i);
            multiplier += exp;
            velocity += exp*previous_odoms_[i].twist.twist.linear.x;
        }
        velocity /= multiplier;

        enemy_odom.twist.twist.linear.x = velocity;

        float yaw;
        // If the velocity is greater than a threshold then we calculate a
        // new orientation
        if(velocity > VEL_THRESH) {
            // Take the change in position and calculate the orientation
            // assuming that the enemy moves in a straight line
            yaw = atan2(dy, dx);
        }
        else {
            // We don't have enough data to estimate the enemy pose so 
            // assume yaw is unchanged.
            yaw = 2*acos(previous_odoms_.back().pose.pose.orientation.w);
        }
        multiplier = 1;
        // Moving average of previous yaw estimates
        for(int i = 0; i < previous_odoms_.size(); i++) {
            float exp = 1.0 / pow(2, previous_odoms_.size() - i);
            multiplier += exp;
            yaw += exp*2*acos(previous_odoms_[i].pose.pose.orientation.w);
        }
        yaw /= multiplier;
        enemy_odom.pose.pose.orientation.z = sin(yaw/2);
        enemy_odom.pose.pose.orientation.w = cos(yaw/2);
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

    previous_odoms_.push_back(enemy_odom);
    while(previous_odoms_.size() > ODOMETRY_MEMORY)
        previous_odoms_.erase(previous_odoms_.begin());
}
