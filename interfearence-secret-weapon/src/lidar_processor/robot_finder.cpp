#include "lidar_processor/robot_finder.hpp"

RobotFinder::RobotFinder(std::string laser_topic, 
                         float arena_diameter,
                         float max_robot_side,
                         float min_robot_side,
                         float object_distance_threshold,
                         float velocity_threshold,
                         int odometry_memory) 
        :arena_diameter_(arena_diameter), max_robot_side_(max_robot_side),
         min_robot_side_(min_robot_side), 
         object_distance_threshold_(object_distance_threshold),
         velocity_threshold_(velocity_threshold), 
         odometry_memory_(odometry_memory), laser_topic_(laser_topic)
{
    calibration_data_size_ = 0;
    // Start with the calibration callback
    laser_sub_ = nh_.subscribe(laser_topic,
                               50,
                               &RobotFinder::calibration_cb,
                               this);
    reset_sub_ = nh_.subscribe("/reset", 2, &RobotFinder::reset_cb, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("enemy_vo", 50);
    debug_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>(
        "laser_scan_point_cloud", 50);

    nh_.getParam("tf_prefix", tf_prefix);
}

RobotFinder::~RobotFinder() {
    // dtor
}

void RobotFinder::calibrate() {
    while(calibration_data_size_ < 10) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    // Loop through each ray and if the distance is less than 
    // CALIBRATION_THRESHOLD more than 100*RATIO% of the time then count it
    // as a location on the robot
    const float CALIBRATION_THRESHOLD = 0.1;
    const float RATIO = 0.85;
    std::vector<float> thresholds;
    thresholds.resize(calibration_data_[0].size());
    for(auto vec : calibration_data_) {
        for(int i = 0; i < vec.size(); i++) {
            if(vec[i] > CALIBRATION_THRESHOLD || std::isnan(vec[i])) {
                thresholds[i] += 1.0 / calibration_data_.size();
            }
        }
    }
    for(int i = 0; i < thresholds.size(); i++) {
        if(thresholds[i] < RATIO) {
            ignore_scans_.push_back(i);
            ROS_INFO_STREAM(i);
        }
    }

    ROS_INFO_STREAM("Calibration completed");

    // Switch back to the normal callback
    laser_sub_ = nh_.subscribe(laser_topic_, 
                               50, 
                               &RobotFinder::laserscan_cb, 
                               this);
}

void RobotFinder::calibration_cb(sensor_msgs::LaserScan::ConstPtr msg) {
    calibration_data_size_++;
    calibration_data_.push_back({});
    for(auto range : msg->ranges)
        calibration_data_.back().push_back(range);
}

void RobotFinder::laserscan_cb(sensor_msgs::LaserScan::ConstPtr msg) {
    // Calculate 3d points relative to the laser scanner frame
    std::vector<Point> detections;
    float angle = msg->angle_min;
    float scans = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    for(int i = 0; i < scans; i++) {
        float dist = msg->ranges[i];
        if(std::find(ignore_scans_.begin(), 
                     ignore_scans_.end(), i) == ignore_scans_.end() &&
           msg->range_min < dist && dist < msg->range_max
           && dist < arena_diameter_)
        {
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
        ROS_WARN_STREAM_THROTTLE(
            1, "[" << tf_prefix << "] No groups of points found!!!");
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

    sensor_msgs::PointCloud cloud_msg;
    cloud_msg.header = msg->header;
    sensor_msgs::ChannelFloat32 channel;
    channel.name = "intensity";
    for(Point p : enemy_positions) {
        geometry_msgs::Point32 gm_p;
        gm_p.x = p.x; gm_p.y = p.y; gm_p.z = p.z;
        cloud_msg.points.push_back(gm_p);
        channel.values.push_back(sqrt(p.x*p.x + p.y*p.y));
    }
    cloud_msg.channels.push_back(channel);
    debug_point_cloud_pub_.publish(cloud_msg);

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
        int best_p = 0;
        int p = 0;

        for(const Point& point : enemy_positions) {
            if(position_weigh(point) < best_weight) {
                best_point = point;
                best_p = p;
            }
            p++;
        }

        // Find the contours of the group
        std::vector<Point> contours = graham_scan(remaining_groups[best_p]);

        enemy_odom.pose.pose.position.x = best_point.x;
        enemy_odom.pose.pose.position.y = best_point.y;
        // Enemy hasn't started moving yet, so assume they are facing us
        // Find the edge that is most perpendicular to us and set the
        // normal of that to be the forwards vector of the enemy
        float x1, x2, y1, y2, dot, best_dot, mag;
        x1 = best_point.x;
        y1 = best_point.y;
        mag = sqrt(x1*x1 + y1*y1);
        x1 /= mag;
        y1 /= mag;
        x2 = contours.back().x - contours[0].x;
        y2 = contours.back().y - contours[0].y;
        mag = sqrt(x2*x2 + y2*y2);
        x2 /= mag;
        y2 /= mag;
        best_dot = fabs(x1*x2 + y1*y2);
        // contours[a] - contours[b] ...
        int a, b;
        a = contours.size() - 1;
        b = 0;
        for(int i = 1; i < contours.size(); i++) {
            x2 = contours[i-1].x - contours[i].x;
            y2 = contours[i-1].y - contours[i].y;
            mag = sqrt(x2*x2 + y2*y2);
            x2 /= mag;
            y2 /= mag;
            dot = fabs(x1*x2 + y1*y2);
            if(dot < best_dot) {
               best_dot = dot;
               a = i-1;
               b = i;
            }
        }

        // We now know the best edge, find the normal to it and calculate 
        // yaw
        // yaw will be 180 + asin(1 - dot^2) * sign_of_dot
        x2 = contours[a].x - contours[b].x;
        y2 = contours[a].y - contours[b].y;
        mag = sqrt(x2*x2 + y2*y2);
        x2 /= mag;
        y2 /= mag;
        dot = x2*x2 + y2*y2;
        float yaw = 3.1415926 + asin(1 - dot*dot) * ((dot < 0) ? -1 : 1);

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
        int p = 0, best_p = 0;

        for(const Point& point : enemy_positions) {
            if(position_weigh(point) < best_weight) {
                best_point = point;
                best_p = p;
            }
            p++;
        }

        // Find the contours of the group
        std::vector<Point> contours = graham_scan(remaining_groups[best_p]);

        enemy_odom.pose.pose.position.x = best_point.x;
        enemy_odom.pose.pose.position.y = best_point.y;
        // Enemy has started moving, so find the edge most similar to the
        // previous detected orientation
        float x1, x2, y1, y2, dot, mag, prev_yaw, best_yaw, yaw;
        prev_yaw = 2*acos(previous_odoms_.back().pose.pose.orientation.w);
        x1 = best_point.x;
        y1 = best_point.y;
        mag = sqrt(x1*x1 + y1*y1);
        x1 /= mag;
        y1 /= mag;
        x2 = contours.back().x - contours[0].x;
        y2 = contours.back().y - contours[0].y;
        mag = sqrt(x2*x2 + y2*y2);
        x2 /= mag;
        y2 /= mag;
        dot = x1*x2 + y1*y2;
        best_yaw = 3.1415926 + asin(1 - dot*dot) * ((dot < 0) ? -1 : 1);
        for(int i = 1; i < contours.size(); i++) {
            x2 = contours[i-1].x - contours[i].x;
            y2 = contours[i-1].y - contours[i].y;
            mag = sqrt(x2*x2 + y2*y2);
            x2 /= mag;
            y2 /= mag;
            dot = x1*x2 + y1*y2;
            yaw =  3.1415926 + asin(1 - dot*dot) * ((dot < 0) ? -1 : 1);
            if(fabs(yaw - prev_yaw) < fabs(best_yaw - prev_yaw))
               best_yaw = yaw;
        }

        enemy_odom.pose.pose.orientation.z = sin(best_yaw/2);
        enemy_odom.pose.pose.orientation.w = cos(best_yaw/2);

        // We can now estimate the velocity as the delta position
        float dx = best_point.x - prev.x;
        float dy = best_point.y - prev.y;
        float velocity = sqrt(pow(dx / dt, 2) + pow(dy / dt, 2));
        enemy_odom.twist.twist.linear.x = velocity;
//        enemy_odom.twist.twist.linear.x = (best_point.x - prev.x) / dt;
//        enemy_odom.twist.twist.linear.y = (best_point.y - prev.y) / dt;
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
        int p = 0, best_p = 0;

        for(const Point& point : enemy_positions) {
            if(position_weigh(point) < best_weight) {
                best_point = point;
                best_p = p;
            }
            p++;
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

        // Find the contours of the group
        std::vector<Point> contours = graham_scan(remaining_groups[best_p]);

        enemy_odom.pose.pose.position.x = best_point.x;
        enemy_odom.pose.pose.position.y = best_point.y;
        // Enemy has started moving, so find the edge most similar to the
        // previous detected orientation
        float x1, x2, y1, y2, dot, mag, prev_yaw, best_yaw, yaw;
        prev_yaw = 2*acos(previous_odoms_.back().pose.pose.orientation.w);
        x1 = best_point.x;
        y1 = best_point.y;
        mag = sqrt(x1*x1 + y1*y1);
        x1 /= mag;
        y1 /= mag;
        x2 = contours.back().x - contours[0].x;
        y2 = contours.back().y - contours[0].y;
        mag = sqrt(x2*x2 + y2*y2);
        x2 /= mag;
        y2 /= mag;
        dot = x1*x2 + y1*y2;
        best_yaw = 3.1415926 + asin(1 - dot*dot) * ((dot < 0) ? -1 : 1);
        for(int i = 1; i < contours.size(); i++) {
            x2 = contours[i-1].x - contours[i].x;
            y2 = contours[i-1].y - contours[i].y;
            mag = sqrt(x2*x2 + y2*y2);
            x2 /= mag;
            y2 /= mag;
            dot = x1*x2 + y1*y2;
            yaw =  3.1415926 + asin(1 - dot*dot) * ((dot < 0) ? -1 : 1);
            if(fabs(yaw - prev_yaw) < fabs(best_yaw - prev_yaw))
               best_yaw = yaw;
        }

        enemy_odom.pose.pose.orientation.z = sin(best_yaw/2);
        enemy_odom.pose.pose.orientation.w = cos(best_yaw/2);
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
    while(previous_odoms_.size() > odometry_memory_)
        previous_odoms_.erase(previous_odoms_.begin());
}

// From https://www.tutorialspoint.com/cplusplus-program-to-implement-graham-scan-algorithm-to-find-the-convex-hull
std::vector<Point> RobotFinder::graham_scan(const std::vector<Point>& in) {
    static auto secondTop = [](std::stack<Point>& stk) {
        Point tmp = stk.top();
        stk.pop();
        Point res = stk.top();
        stk.push(tmp);
        return res;
    };

    static auto squaredDist = [](const Point& p1, const Point& p2) {
        return (p1.x-p2.x) * (p1.x-p2.x) + (p1.y-p2.y) * (p1.y-p2.y);
    };

    static auto direction = []
        (const Point& p0, const Point& p1, const Point& p2) 
    {
        float val = (p1.y-p0.y) * (p2.x-p1.x) - (p1.x-p0.x) * (p2.y-p1.y);
        if(fabs(val) <= 1e-5) return 0; // colinear
        else if(val < 0) return 2; // anticlockwise
        return 1; // clockwise
    };


    static auto comp = [direction, squaredDist](
        const Point& p0, const Point& p1, const Point& p2)
    {
        int dir = direction(p0, p1, p2);
        if(!dir)
            return (squaredDist(p0, p2) > squaredDist(p0, p1))?false:true;
        return (dir==2) ? false : true;
    };

    std::vector<Point> hull_pts, points;
    points = in;

    float minY = points[0].y, min = 0;
    for(int i = 0; i < points.size(); i++) {
        int y = points[i].y;
        if((y < minY) || (minY == y) && points[i].x < points[min].x) {
            minY = points[i].y;
            min = i;
        }
    }
    std::swap(points[0], points[min]);
    std::sort(points.begin()+1, points.end(), 
              [comp, points](const Point& a, const Point& b) { 
                  return comp(points[0], a, b); 
              });
    int arrSize = 1;
    for(int i = 1; i < points.size(); i++) {
        while(i < points.size()-1 &&
              direction(points[0], points[i], points[i+1]) == 0) i++;
        points[arrSize] = points[i];
        arrSize++;
    }

    if(arrSize < 3)
        return hull_pts; // Less than 3 points is invalid
    std::stack<Point> stk;
    stk.push(points[0]);
    stk.push(points[1]);
    stk.push(points[2]);
    for(int i = 3; i < arrSize; i++) {
        while(direction(secondTop(stk), stk.top(), points[i]) !=2) {
            if(stk.size() == 2) break;
            stk.pop();
        }
        stk.push(points[i]);
    }
    while(!stk.empty()) {
        hull_pts.push_back(stk.top());
        stk.pop();
    }

    return hull_pts;
}

void RobotFinder::reset_cb(std_msgs::Bool::ConstPtr msg) {
    // We want to reset the scanner when the robot is told to launch again
    if(msg->data) {
        previous_odoms_.clear();
    }
}
