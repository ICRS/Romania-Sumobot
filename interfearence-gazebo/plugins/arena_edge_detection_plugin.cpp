#include "arena_edge_detection_plugin.hpp"

namespace gazebo {
    GZ_REGISTER_MODEL_PLUGIN(ArenaEdgeDetectionPlugin);

    ArenaEdgeDetectionPlugin::ArenaEdgeDetectionPlugin()
        :nh_("arena_edge_detection") 
    {
        // ctor
    }

    ArenaEdgeDetectionPlugin::~ArenaEdgeDetectionPlugin() {
        ROS_DEBUG_STREAM_NAMED("ArenaEdgeDetectionPlugin", "unloaded");
    }

    void ArenaEdgeDetectionPlugin::Load(
        physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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

        std::string linkName;
        if(_sdf->HasElement("linkName"))
            linkName = _sdf->Get<std::string>("linkName");
        else {
            ROS_FATAL_STREAM_NAMED("ArenaEdgeDetectionPlugin",
                                   "'linkName' has not been set!");
            return;
        }

        this->model = _parent; 

        pub_ = nh_.advertise<interfearence_msgs::EdgeDetection>(
            topicname, 1);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ArenaEdgeDetectionPlugin::OnUpdate,this));

        this->link = this->model->GetLink(linkName);
        if(!link) {
            ROS_FATAL_STREAM(linkName << " does not exist!");
            return;
        }
        last_update = ros::Time::now();
    }

    void ArenaEdgeDetectionPlugin::OnUpdate()
    {
        if((ros::Time::now()-last_update).nsec < 5e7)
            return;

        last_update = ros::Time::now();

        interfearence_msgs::EdgeDetection msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_;

        auto sensorPose = this->link->WorldPose();
        x = sensorPose.Pos().X();
        y = sensorPose.Pos().Y();

        double dist = sqrt(x*x + y*y);
        
        if(dist < 0.67)
            msg.at_edge = false;
        else
            msg.at_edge = true;
        
        pub_.publish(msg);
    }
}
