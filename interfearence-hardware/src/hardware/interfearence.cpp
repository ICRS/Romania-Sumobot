#include "hardware/interfearence.hpp"

Interfearence::Interfearence() {
    // Connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_l("Left", 
                                                        &pos[0],
                                                        &vel[0],
                                                        &eff[0]);
    jnt_state_interface.registerHandle(state_handle_l);

    hardware_interface::JointStateHandle state_handle_r("Right", 
                                                        &pos[1],
                                                        &vel[1],
                                                        &eff[1]);
    jnt_state_interface.registerHandle(state_handle_r);

    registerInterface(&jnt_state_interface);

    // Connect and register the joint position interface
    hardware_interface::JointHandle vel_handle_l(
        jnt_state_interface.getHandle("Left"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_l);

    hardware_interface::JointHandle vel_handle_r(
        jnt_state_interface.getHandle("Right"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_r);

    registerInterface(&jnt_vel_interface);
}

void Interfearence::write() {
    ROS_ERROR_THROTTLE(30, "Interfearence::Write() is not implemented!");
}

void Interfearence::read() {
    ROS_ERROR_THROTTLE(30, "Interfearence::Read() is not implemented!");
}
