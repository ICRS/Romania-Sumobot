#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <time.h>

#include "hardware/interfearence.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_interface");
    
    // Create robot instance
    Interfearence robot;
    // Create controller manager instance
    controller_manager::ControllerManager cm(&robot);

    ros::NodeHandle nh;

    ros::Duration elapsed_time;
    struct timespec last_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &last_time);

    // 100 Hz update rate
    ros::Rate sleeper(100);

    while(ros::ok()) {
        // Calculate dt
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        elapsed_time = 
            ros::Duration(current_time.tv_sec - last_time.tv_sec
                        + (current_time.tv_nsec - last_time.tv_nsec) 
                        / 1000000000.0);
        last_time = current_time;
        // Read the current robot joint states
        robot.read();
        cm.update(ros::Time::now(), elapsed_time);
        // Publish new commands to robot
        robot.write();

        // Process callbacks
        ros::spinOnce();
        sleeper.sleep();
    }
}
