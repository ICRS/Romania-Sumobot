#define PY_SSIZE_T_CLEAN
#include <python3.6m/Python.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <chrono>

class Interfearence : public hardware_interface::RobotHW {
public:
    Interfearence();
    ~Interfearence();
    void write();
    void read();
private:
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;
    double cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];

    std::chrono::steady_clock::time_point last_time;

    PyObject *odrive_interface_;

    void set_wheel_vel(int axis, double velocity);
    double get_wheel_vel(int axis, double dt);
    double get_wheel_eff(int axis);
    double get_battery_voltage();
};
