#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <chrono>

class Interfearence : public hardware_interface::RobotHW {
public:
    Interfearence();
    void write();
    void read();
private:
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::EffortJointInterface jnt_eff_interface_;
    double cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];

    std::chrono::steady_clock::time_point last_time;

    float radians_per_pulse_;
};
