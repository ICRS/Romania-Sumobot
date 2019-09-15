#ifndef __ARENA_EDGE_DETECTION_PLUGIN_HPP__
#define __ARENA_EDGE_DETECTION_PLUGIN_HPP__

#include <string>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/CameraPlugin.hh>

#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <gazebo_plugins/gazebo_ros_camera.h>

#include <ros/ros.h>
#include <interfearence_msgs/EdgeDetection.h>

namespace gazebo {
    class ArenaEdgeDetectionPlugin : 
        public CameraPlugin, GazeboRosCameraUtils 
    {
    public:
        ArenaEdgeDetectionPlugin();
        virtual ~ArenaEdgeDetectionPlugin();

        // Load the plugin
        void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    protected:    
        // Update on new frame
        virtual void OnNewFrame(
            const unsigned char *_image, unsigned int _width, 
            unsigned int _height, unsigned int _depth,
            const std::string &_format);

        ros::NodeHandle nh_;
        ros::Publisher pub_;

        // Above what intensity is considered white?
        float upper_threshhold_;
        // Below what intensity is considered black?
        float lower_threshhold_;

        // TF coordinate frame
        std::string frame_;

        // Store the previous value in case we're in the Schmidt region
        bool previous_result_;
    };
}

#endif // __ARENA_EDGE_DETECTION_PLUGIN_HPP__
