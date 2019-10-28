#define PY_SSIZE_T_CLEAN
#include <python3.6m/Python.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <chrono>
#include <exception>

enum class EdgeSensor {
    FRONT_LEFT,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT,
};

class Interfearence : public hardware_interface::RobotHW {
public:
    // Ctor
    Interfearence();
    // Dtor
    ~Interfearence();
    
    // Write current control values to the robot
    void write();
    // Read current control values from the robot
    void read();
    // Check the state of the reset pins
    bool check_reset_state();
    // Read an edge sensor state
    bool read_edge_sensor(EdgeSensor sensor) { 
        return this->edges_[sensor]; 
    };
    // Check whether the Odrive has thrown an error
    bool check_errors();
    // Clear an error thrown by the odrive
    void clear_errors();
    // Called by the main function as it takes a while and causes ROS
    // Control to fail to find the controller
    void setup_odrive();
private:
    // Setup functions
    void setup_ros_control();
    void setup_gpio();

    // ROS Control variables
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::EffortJointInterface jnt_eff_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;
    double cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];

    // Time keeping
    std::chrono::steady_clock::time_point last_time;

    // Pointer to the python odrive_interface class
    PyObject *odrive_interface_;

    // Function wrappers for python call to the odrive
    void set_wheel_vel(int axis, double velocity);
    void set_wheel_eff(int axis, double eff);
    double get_wheel_vel(int axis, double dt);
    double get_wheel_eff(int axis);
    double get_battery_voltage();
    void reset();
    void release_reset();

    /// GPIO STUFF

    // Reset module stuff
    bool prev_reset_state_;
    int start_pin_;
    int kill_pin_;

    // Electromagnet stuff
    int electromagnet_pin_;

    // Neopixel stuff
    int neopixel_pin_;

    // Edge sensor stuff
    std::map<EdgeSensor, int> edge_out_pins_;
    std::map<EdgeSensor, int> edge_vcc_pins_;
    std::map<
        EdgeSensor, std::chrono::time_point<std::chrono::steady_clock>> 
            edge_drop_times_;
    std::map<EdgeSensor, int> white_thresholds_;
    std::map<EdgeSensor, int> black_thresholds_;
    std::map<EdgeSensor, bool> edges_;
    // Callback function to regularly check the states of the edge
    // sensors
    void edge_sensor_cb(EdgeSensor sensor);
    struct EdgeCbHelper {
        Interfearence *interfearence;
        EdgeSensor sensor;
    };
    std::map<EdgeSensor, EdgeCbHelper> edge_cb_helpers_;

    friend void edge_sensor_cb_wrapper(
        int gpio, int edge, uint32_t tick, void *edge_cb_helper);
};
