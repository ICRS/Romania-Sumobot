#include "arena_edge_detection_plugin.hpp"

namespace gazebo {
    GZ_REGISTER_SENSOR_PLUGIN(ArenaEdgeDetectionPlugin);

    ArenaEdgeDetectionPlugin::ArenaEdgeDetectionPlugin()
        :nh_("arena_edge_detection"), previous_result_(false)
    {
        // ctor
    }

    ArenaEdgeDetectionPlugin::~ArenaEdgeDetectionPlugin() {
        ROS_DEBUG_STREAM_NAMED("ArenaEdgeDetectionPlugin", "unloaded");
    }

    void ArenaEdgeDetectionPlugin::Load(
        sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        if(!ros::isInitialized()) {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialised!!! Unable to load plugin.");
            return;
        }

        // Read parameters
        if(_sdf->HasElement("frameName"))
            frame_ = _sdf->Get<std::string>("frameName");
        else {
            ROS_FATAL_STREAM_NAMED("ArenaEdgeDetectionPlugin", 
                                   "'frameName' has not been set!");
            return;
        }
        std::string topicname;
        if(_sdf->HasElement("topicName"))
            topicname = _sdf->Get<std::string>("topicName");
        else {
            ROS_FATAL_STREAM_NAMED("ArenaEdgeDetectionPlugin", 
                                   "'topicName' has not been set!");
            return;
        }
        if(_sdf->HasElement("upperThreshhold"))
            upper_threshhold_ = _sdf->Get<float>("upperThreshhold");
        else {
            ROS_FATAL_STREAM_NAMED("ArenaEdgeDetectionPlugin", 
                                   "'upperThreshhold' has not been set!");
            return;
        }
        if(_sdf->HasElement("lowerThreshhold"))
            lower_threshhold_ = _sdf->Get<float>("lowerThreshhold");
        else {
            ROS_FATAL_STREAM_NAMED("ArenaEdgeDetectionPlugin", 
                                   "'lowerThreshhold' has not been set!");
            return;
        }

        CameraPlugin::Load(_parent, _sdf);

         // copying from CameraPlugin into GazeboRosCameraUtils
        this->parentSensor_ = this->parentSensor;
        this->width_ = this->width;
        this->height_ = this->height;
        this->depth_ = this->depth;
        this->format_ = this->format;
        this->camera_ = this->camera;

        GazeboRosCameraUtils::Load(_parent, _sdf);
        
        pub_ = nh_.advertise<interfearence_msgs::EdgeDetection>(
            topicname, 1);

        this->parentSensor_->SetActive(true);
        this->parentSensor->SetActive(true);

        ROS_INFO_NAMED(_parent->Name(), "successfully loaded.");
    }

    void ArenaEdgeDetectionPlugin::OnNewFrame(
        const unsigned char *_image, unsigned int _width,
        unsigned int _height, unsigned int _depth,
        const std::string &_format)
    {
        interfearence_msgs::EdgeDetection msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_;

        float avg_brightness;
        for(unsigned int w = 0; w < _width; w++) {
            for(unsigned int h = 0; h < _height; h++) {
                avg_brightness += _image[_height*w+h] / 255.f;
            }
        }
        
        avg_brightness /= _width * _height;
        if(avg_brightness > upper_threshhold_)
            previous_result_ = true;
        else if(avg_brightness < lower_threshhold_)
            previous_result_ = false;
        msg.at_edge = previous_result_;

        pub_.publish(msg);
    }
}
