#ifndef __ARENA_EDGE_DETECTION_PLUGIN_HPP__
#define __ARENA_EDGE_DETECTION_PLUGIN_HPP__

#include <string>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <interfearence_msgs/EdgeDetection.h>

namespace gazebo {
    class ArenaEdgeDetectionPlugin : 
        public ModelPlugin 
    {
    public:
        ArenaEdgeDetectionPlugin();
        virtual ~ArenaEdgeDetectionPlugin();

        // Load the plugin
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:    
        // Update on new frame
        virtual void OnUpdate();

        ros::NodeHandle nh_;
        ros::Publisher pub_;

        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        double x,y;
        physics::LinkPtr link;
        
        // TF coordinate frame
        std::string frame_;

        ros::Time last_update;
    };
}

#endif // __ARENA_EDGE_DETECTION_PLUGIN_HPP__
