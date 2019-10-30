#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <controller_manager/controller_manager.h>

#include <time.h>

#include "interfearence_hardware/interfearence.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_interface");
    
    ros::NodeHandle nh;

    // Create reset publisher and ensure that we're in the reset state
    ros::Publisher reset_pub = nh.advertise<std_msgs::Bool>("/reset", 2);
    std_msgs::Bool reset_msg;
    reset_msg.data = true;
    reset_pub.publish(reset_msg);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Create robot instance
    Interfearence robot;

    // Create controller manager instance
    controller_manager::ControllerManager cm(&robot, nh);

    // Setup the odrive
    robot.setup_odrive();

    ros::Duration elapsed_time;
    struct timespec last_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &last_time);

    // 100 Hz update rate
    ros::Rate sleeper(1000);

    while(ros::ok()) {
        // Calculate dt - intentionally not using ROS time as we want
        // this to be as accurate as possible
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        elapsed_time = 
            ros::Duration((current_time.tv_sec - last_time.tv_sec)
                        + (current_time.tv_nsec - last_time.tv_nsec) / 1e9);
        last_time = current_time;
        // Read the current robot joint states
        robot.read();
        cm.update(ros::Time::now(), elapsed_time);
        // Publish new commands to robot
        robot.write();

        // Check the reset state and publish it
        reset_msg.data = robot.check_reset_state();
        reset_pub.publish(reset_msg);

        ROS_WARN_STREAM_THROTTLE(5, "dt: " << elapsed_time);

        if(robot.check_errors()) {
            // Not sure what to do? For now just sleep for 0.2s and 
            // then clear the error
            ros::Duration(0.2).sleep();
            robot.clear_errors();
        }

        sleeper.sleep();
    }
}
