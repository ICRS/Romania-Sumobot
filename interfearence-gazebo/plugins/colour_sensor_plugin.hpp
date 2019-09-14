#ifndef __COLOUR_SENSOR_PLUGIN_HPP__
#define __COLOUR_SENSOR_PLUGIN_HPP__

#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo {

class ColourSensorPlugin :public SensorPlugin {
public:
    ColourSensorPlugin();
    virtual ~ColourSensorPlugin();

    virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

    virtual void OnUpdate();
private:
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;

    rendering::CameraPtr parent_camera_;

    event::ConnectionPtr update_connection_;
};

}

#endif // __COLOUR_SENSOR_PLUGIN_HPP__
