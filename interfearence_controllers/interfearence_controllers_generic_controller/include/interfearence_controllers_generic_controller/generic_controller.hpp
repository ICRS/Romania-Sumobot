#ifndef __GENERIC_CONTROLLER_HPP__
#define __GENERIC_CONTROLLER_HPP__

#include <ros/ros.h>
#include <std_msgs/Bool.h>

// An abstract, generic controller with other controllers can inherit from.
// Just fill in the virtual functions and add methods of your own.
class GenericController {
public:
    GenericController(int argc, char **argv);
    virtual ~GenericController();

    // The main ros loop. Call this from your main function.
    void main();

protected:
    // Reset function needs to be filled in with initialisation stuff.
    // Don't forget to reset any time-based things (e.g. TF) as Gazebo
    // will break you if you don't!
    virtual void reset() = 0;

    // Fill this in with your (non-blocking) control loop.
    virtual void update() = 0;

    // Name of the node. Replace this with what you want it to be called.
    virtual std::string get_name();

    // Updates the RESET variable below
    void reset_callback(std_msgs::Bool::ConstPtr msg);

    /*************
     * IMPORTANT *
     *************/
    // When this is true your node will run the reset function instead
    // of the update function.
    bool RESET;

    // ROS nodehandle - feel free to use
    ros::NodeHandle nh_;

    // Default update rate of the main loop is 50 Hz. Change this in your
    // constructor if you want a different update frequency.
    ros::Rate rate_;

    // Subscriber to the reset topic
    ros::Subscriber reset_sub_;
};

#endif // __GENERIC_CONTROLLER_HPP__
