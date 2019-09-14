#include "colour_sensor_plugin.hpp"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ColourSensorPlugin)

ColourSensorPlugin::ColourSensorPlugin() :SensorPlugin() {
    // ctor
}

ColourSensorPlugin::~ColourSensorPlugin() {
    // dtor
}

void ColourSensorPlugin::Load(
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) 
{
    this->parent_camera_ = 
        std::dynamic_pointer_cast<rendering::Camera>(sensor);
    
    if(!this->parent_camera_) {
        gzerr << "ColourSensorPlugin requires a Camera.\n";
        return;
    }

    std::string topic;
    if(sdf->HasElement("topicName"))
        topic = sdf->Get<std::string>("topicName");
    else {
        gzerr << "ColourSensorPlugin requires a topicName.\n";
        return;
    }
    this->state_pub_ = nh_.advertise<std_msgs::Bool>(topic, 1);

    this->update_connection_ = this->parent_camera_->ConnectUpdated(
        std::bind(&ColourSensorPlugin::OnUpdate, this));

    this->parent_sensor_->setImageSize(1, 1);
    this->parent_sensor_->SetActive(true);
}

void ColourSensorPlugin::OnUpdate() {
    
}
