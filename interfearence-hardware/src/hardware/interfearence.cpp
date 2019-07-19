#include "hardware/interfearence.hpp"

Interfearence::Interfearence() {
    // Connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_l("left_wheel_joint", 
                                                        &pos[0],
                                                        &vel[0],
                                                        &eff[0]);
    jnt_state_interface_.registerHandle(state_handle_l);

    hardware_interface::JointStateHandle state_handle_r("right_wheel_joint", 
                                                        &pos[1],
                                                        &vel[1],
                                                        &eff[1]);
    jnt_state_interface_.registerHandle(state_handle_r);

    registerInterface(&jnt_state_interface_);

    // Connect and register the joint position interface
    hardware_interface::JointHandle vel_handle_l(
        jnt_state_interface_.getHandle("left_wheel_joint"), &cmd[0]);
    jnt_eff_interface_.registerHandle(vel_handle_l);

    hardware_interface::JointHandle vel_handle_r(
        jnt_state_interface_.getHandle("right_wheel_joint"), &cmd[1]);
    jnt_eff_interface_.registerHandle(vel_handle_r);

    registerInterface(&jnt_eff_interface_);
}

void Interfearence::write() {
    ROS_ERROR_THROTTLE(10, "Interfearence::write() is not implemented!");
    // Write the PWM signal strength to the motors
    // motor_left.pwm(cmd[0] / max_cmd[0]);
    // motor_right.pwm(cmd[1] / max_cmd[1]);
    eff[0] = cmd[0];
    eff[1] = cmd[1];
}

void Interfearence::read() {
    ROS_ERROR_THROTTLE(10, "Interfearence::read() is not implemented!");
    // Read pulses since last read
    unsigned int new_counts[2];
    // motor_left.read_new_counts(&new_counts[0]);
    // motor_right.read_new_counts(&new_counts[1]);
    
    // Get delta time
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dt = now - last_time;
    last_time += dt;

    vel[0] = radians_per_pulse_ * new_counts[0] / dt;
    vel[1] = radians_per_pulse_ * new_counts[1] / dt;

    pos[0] += radians_per_pulse_ * new_counts[0];
    pos[1] += radians_per_pulse_ * new_counts[1];

    // Publish odometry
}
