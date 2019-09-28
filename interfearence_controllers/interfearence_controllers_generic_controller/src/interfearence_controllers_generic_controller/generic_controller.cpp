#include "interfearence_controllers_generic_controller/generic_controller.hpp"

GenericController::GenericController(int argc, char**argv) 
    :RESET(true), rate_(50) 
{
    ros::init(argc, argv, this->get_name());

    this->reset_sub_ = this->nh_.subscribe(
        "/reset", 1, &GenericController::reset_callback, this);
}

GenericController::~GenericController() {
    // dtor
}

void GenericController::main() {
    while(ros::ok()) {
        if(this->RESET)
            this->reset();
        else
            this->update();
        
        this->rate_.sleep();
    }
}

void GenericController::reset_callback(std_msgs::Bool::ConstPtr msg) {
    this->RESET = msg->data;
}

std::string GenericController::get_name() {
    return "generic_controller";
}
