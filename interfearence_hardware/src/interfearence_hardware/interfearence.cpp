#include "interfearence_hardware/interfearence.hpp"

#include <thread>

// Raspberry Pis run on arm processors, while our development computers
// don't. This way anything Pi specific won't run (or be compiled) on
// desktops
#ifdef __arm__
#include<pigpio.h>
#endif // __arm__

/// PIN NUMBERS

// Electromagnet
#define ELECTROMAGNET 7

// Neopixel
#define NEOPIXEL_PIN  18

// Edge sensors
#define TOP_LEFT_OUT  26
#define TOP_LEFT_VCC  19
#define TOP_RIGHT_OUT 21 
#define TOP_RIGHT_VCC 20
#define BOT_LEFT_OUT  13
#define BOT_LEFT_VCC  6
#define BOT_RIGHT_OUT 16
#define BOT_RIGHT_VCC 12

// Start module
#define START_PIN     25
#define KILL_PIN      8

Interfearence::Interfearence() {
    /// SETUP ROS CONTROL STUFF

    // Connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_l("left_wheel_joint", 
                                                        &pos[1],
                                                        &vel[1],
                                                        &eff[1]);
    jnt_state_interface_.registerHandle(state_handle_l);

    hardware_interface::JointStateHandle state_handle_r("right_wheel_joint", 
                                                        &pos[0],
                                                        &vel[0],
                                                        &eff[0]);
    jnt_state_interface_.registerHandle(state_handle_r);

    registerInterface(&jnt_state_interface_);

    // Connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_l(
        state_handle_l, &cmd[1]);
    jnt_vel_interface_.registerHandle(vel_handle_l);

    hardware_interface::JointHandle vel_handle_r(
        state_handle_r, &cmd[0]);
    jnt_vel_interface_.registerHandle(vel_handle_r);

    registerInterface(&jnt_vel_interface_);

    // Connect and register the joint effort interface
    hardware_interface::JointHandle eff_handle_l(
        state_handle_l, &cmd[1]);
    jnt_eff_interface_.registerHandle(eff_handle_l);

    hardware_interface::JointHandle eff_handle_r(
        state_handle_r, &cmd[0]);
    jnt_eff_interface_.registerHandle(eff_handle_r);

    registerInterface(&jnt_eff_interface_);

    /// SETUP ODRIVE STUFF

    // Initialise the python interpreter
    Py_Initialize();

    PyObject *module_name = PyUnicode_FromString("interfearence_hardware.odrive_interface");
    PyObject *module = PyImport_Import(module_name);

    if(module == nullptr) {
        PyErr_Print();
        throw std::runtime_error("Failed to import interfearence_hardware.odrive_interface.py");
        return;
    }

    PyObject *dict = PyModule_GetDict(module);
    if(dict == nullptr) {
        PyErr_Print();
        throw std::runtime_error("Failed to import __dict__");
        return;
    }

    PyObject *py_class = PyDict_GetItemString(dict, "OdriveInterface");
    if(PyCallable_Check(py_class))
        odrive_interface_ = PyObject_CallObject(py_class, NULL);
    else {
        throw std::runtime_error("Failed to instantiate OdriveInterface");
        return;
    }

    // Reset the odrive
    this->prev_reset_state_ = false;
    this->reset();

    /// SETUP GPIO STUFF

    this->electromagnet_pin_ = ELECTROMAGNET;
    this->neopixel_pin_ = NEOPIXEL_PIN;
    this->edge_out_pins_[EdgeSensor::FRONT_LEFT] = TOP_LEFT_OUT;
    this->edge_vcc_pins_[EdgeSensor::FRONT_LEFT] = TOP_LEFT_VCC;
    this->edge_out_pins_[EdgeSensor::FRONT_RIGHT] = TOP_RIGHT_OUT;
    this->edge_vcc_pins_[EdgeSensor::FRONT_RIGHT] = TOP_RIGHT_VCC;
    this->edge_out_pins_[EdgeSensor::REAR_LEFT] = BOT_LEFT_OUT;
    this->edge_vcc_pins_[EdgeSensor::REAR_LEFT] = BOT_LEFT_VCC;
    this->edge_out_pins_[EdgeSensor::REAR_RIGHT] = BOT_RIGHT_OUT;
    this->edge_vcc_pins_[EdgeSensor::REAR_RIGHT] = BOT_RIGHT_VCC;
    this->start_pin_ = START_PIN;
    this->kill_pin_ = KILL_PIN;

#ifdef __arm__
    // Initialise PiGPIO
    if(gpioInitialise() < 0) {
        throw std::runtime_error("PiGPIO initialisation failed");
        return;
    }

    gpioSetMode(this->start_pin_, PI_INPUT);
    gpioSetMode(this->kill_pin_, PI_INPUT);
    gpioSetMode(this->electromagnet_pin_, PI_OUTPUT);
    gpioSetMode(this->neopixel_pin_, PI_OUTPUT);

    // Create falling edge callbacks for each edge sensor
    EdgeSensor sensors[4] = {
        EdgeSensor::FRONT_LEFT, EdgeSensor::FRONT_RIGHT,
        EdgeSensor::REAR_LEFT , EdgeSensor::REAR_RIGHT };
    for(EdgeSensor sensor : sensors) {
        this->edge_drop_times_[sensor] = std::chrono::steady_clock::now();
        this->edge_cb_helpers_[sensor].interfearence = this;
        this->edge_cb_helpers_[sensor].sensor = sensor;
        gpioSetMode(this->edge_vcc_pins_[sensor], PI_OUTPUT);
        gpioSetMode(this->edge_out_pins_[sensor], PI_INPUT);
        gpioSetISRFuncEx(
            this->edge_out_pins_[sensor], FALLING_EDGE, 0,
            &edge_sensor_cb_wrapper, 
            (void*)&this->edge_cb_helpers_[sensor]);
        this->read_edge_sensor(sensor);
    }
#endif // __arm__
}

Interfearence::~Interfearence() {
    // Stop the odrive from moving
    this->reset();

    // Uninitialise Python
    Py_Finalize();

#ifdef __arm__    
    // Uninitialise PiGPIO
    gpioTerminate();
#endif // __arm__
}

void Interfearence::write() {
    // Write to the Odrive
    this->set_wheel_vel(0, cmd[0]);
    this->set_wheel_vel(1, cmd[1]);
    //this->set_wheel_eff(0, cmd[0]);
    //this->set_wheel_eff(1, cmd[1]);
    ROS_INFO_STREAM_THROTTLE(
        1,"Setting effort of [" << cmd[0] << "," << cmd[1] << "]");
}

void Interfearence::read() {
    // Get delta time
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = now - last_time;
    last_time = now;

    vel[0] = this->get_wheel_vel(0, dt.count());
    vel[1] = this->get_wheel_vel(1, dt.count());

    pos[0] += vel[0] * dt.count();
    pos[1] += vel[1] * dt.count();

    eff[0] = this->get_wheel_eff(0);
    eff[1] = this->get_wheel_eff(1);

    // TODO: flick an LED?
    ROS_WARN_STREAM_THROTTLE(
        10, "Battery Voltage: " << this->get_battery_voltage());
    ROS_INFO_STREAM_THROTTLE(
        1, "Wheel velocities: " << vel[0] << ", " << vel[1]);
}

void Interfearence::set_wheel_vel(int axis, double vel) {
    // Write the wheel velocities to the Odrive
    PyObject_CallMethod(
        odrive_interface_, "set_wheel_vel", "(i, d)", axis, vel);
}

void Interfearence::set_wheel_eff(int axis, double eff) {
    // Write the wheel efforts(currents) to the Odrive
    PyObject_CallMethod(
        odrive_interface_, "set_wheel_eff", "(i, d)", axis, eff);
}

double Interfearence::get_wheel_vel(int axis, double dt) {
    PyObject *py_vel = PyObject_CallMethod(
        odrive_interface_, "get_wheel_vel", "(i, d)", axis, dt);
    if(!py_vel) {
        PyErr_Print();
        throw std::runtime_error("Failed to get wheel velocity");
        return 0;
    }
    return PyFloat_AsDouble(py_vel);
}

double Interfearence::get_wheel_eff(int axis) {
    PyObject *py_eff = PyObject_CallMethod(
        odrive_interface_, "get_wheel_eff", "(i)", axis);
    if(!py_eff) {
        PyErr_Print();
        throw std::runtime_error("Failed to get wheel effort");
        return 0;
    }
    return PyFloat_AsDouble(py_eff);
}

double Interfearence::get_battery_voltage() {
    PyObject *volt = PyObject_CallMethod(
        odrive_interface_, "get_battery_voltage", NULL);
    if(!volt) {
        PyErr_Print();
        throw std::runtime_error("Failed to get battery voltage");
        return 0;
    }
    return PyFloat_AsDouble(volt);
}

bool Interfearence::check_reset_state() {
    // TODO: Read pins
    bool start_pin = true;
    bool kill_pin = true;
    
    // It's not in the reset state only if both pins are high
    bool state = !(start_pin && kill_pin);

    if(state)
        reset();
    else
        release_reset();

    return state;
}

void Interfearence::reset() {
    if(!prev_reset_state_) {
        PyObject_CallMethod(odrive_interface_, "reset", NULL);
#ifdef __arm__
        // Turn off the electromagnet
        gpioWrite(this->electromagnet_pin_, 0);
#endif // __arm__
        prev_reset_state_ = true;
        ROS_WARN("Entering Reset State");
    }
}

void Interfearence::release_reset() {
    if(prev_reset_state_) {
        PyObject_CallMethod(odrive_interface_, "release_reset", NULL);
#ifdef __arm__
        // Turn on the electromagnet
        gpioWrite(this->electromagnet_pin_, 1);
#endif // __arm__
        prev_reset_state_ = false;
        ROS_WARN("Exiting Reset State");
    }
}

void Interfearence::edge_sensor_cb(EdgeSensor sensor) {
    #ifdef __arm__
    // Calculate time since last callback (i.e. the time for the edge to 
    // fall), and whether we're at edge or not
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = now - this->edge_drop_times_[sensor];
    if(dt.count() > this->black_thresholds_[sensor])
        this->edges_[sensor] = false;
    else if(dt.count() < this->white_thresholds_[sensor])
        this->edges_[sensor] = true;

    // Set the vcc pin to on
    int result = gpioWrite(this->edge_vcc_pins_[sensor], 1);

    // Wait for the out pin to turn on
    result = gpioRead(this->edge_out_pins_[sensor]);
    while(result == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        result = gpioRead(this->edge_out_pins_[sensor]);
    }

    // Set the vcc pin to off and time until the output turns off
    gpioWrite(this->edge_vcc_pins_[sensor], 0);
    this->edge_drop_times_[sensor] = std::chrono::steady_clock::now();
    #endif // __arm__
}

void edge_sensor_cb_wrapper(
        int gpio, int edge, uint32_t tick, void *edge_cb_helper)
{
    Interfearence::EdgeCbHelper *helper = 
        reinterpret_cast<Interfearence::EdgeCbHelper*>(edge_cb_helper);

    helper->interfearence->edge_sensor_cb(helper->sensor);
}
