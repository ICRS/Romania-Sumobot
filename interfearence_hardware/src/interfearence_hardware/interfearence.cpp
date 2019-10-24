#include "interfearence_hardware/interfearence.hpp"

Interfearence::Interfearence() {
    // Initialise the python interpreter
    Py_Initialize();

    PyObject *module_name = PyUnicode_FromString("interfearence_hardware.odrive_interface");
    PyObject *module = PyImport_Import(module_name);

    if(module == nullptr) {
        PyErr_Print();
        throw "Failed to import interfearence_hardware.odrive_interface.py";
        return;
    }

    PyObject *dict = PyModule_GetDict(module);
    if(dict == nullptr) {
        PyErr_Print();
        throw "Failed to import __dict__";
        return;
    }

    PyObject *py_class = PyDict_GetItemString(dict, "OdriveInterface");
    if(PyCallable_Check(py_class))
        odrive_interface_ = PyObject_CallObject(py_class, NULL);
    else {
        throw "Failed to instantiate OdriveInterface";
        return;
    }

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
    jnt_vel_interface_.registerHandle(vel_handle_l);

    hardware_interface::JointHandle vel_handle_r(
        jnt_state_interface_.getHandle("right_wheel_joint"), &cmd[1]);
    jnt_vel_interface_.registerHandle(vel_handle_r);

    registerInterface(&jnt_vel_interface_);
}

Interfearence::~Interfearence() {
    // Uninitialise Python
    Py_Finalize();
}

void Interfearence::write() {
    // Write to the Odrive
    this->set_wheel_vel(0, cmd[0]);
    this->set_wheel_vel(1, cmd[1]);
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

    ROS_INFO_STREAM_THROTTLE(10, "Battery Voltage: " << this->get_battery_voltage());
}

void Interfearence::set_wheel_vel(int axis, double vel) {
    // Write the wheel velocities to the Odrive
    PyObject_CallMethod(
        odrive_interface_, "set_wheel_vel", "(axis, vel)", axis, vel);
}

double Interfearence::get_wheel_vel(int axis, double dt) {
    PyObject *py_vel = PyObject_CallMethod(
        odrive_interface_, "get_wheel_vel", "(axis, dt)", axis, dt);
    if(!py_vel) {
        PyErr_Print();
        throw "Failed to get wheel velocity";
        return 0;
    }
    return PyFloat_AsDouble(py_vel);
}

double Interfearence::get_wheel_eff(int axis) {
    PyObject *py_eff = PyObject_CallMethod(
        odrive_interface_, "get_wheel_eff", "(axis)", axis);
    if(!py_eff) {
        PyErr_Print();
        throw "Failed to get wheel effort";
        return 0;
    }
    return PyFloat_AsDouble(py_eff);
}

double Interfearence::get_battery_voltage() {
    PyObject *volt = PyObject_CallMethod(
        odrive_interface_, "get_battery_voltage", NULL);
    if(!volt) {
        PyErr_Print();
        throw "Failed to get battery voltage";
        return 0;
    }
    return PyFloat_AsDouble(volt);
}
