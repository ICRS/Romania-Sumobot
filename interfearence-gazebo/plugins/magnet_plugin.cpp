#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

// We count this distance as perfect contact between the magnet and the 
// arena
#define ZERO_DISTANCE 0.025

// The (z) position of the magnet relative to the centre of the robot
#define MAGNET_Z_POS -0.0025

// Newtons
#define MAGNET_FORCE 500

// 1 + MAGNET_CONST * (magnet half strength distance in mm)^5 = 2
#define MAGNET_CONST 96.0

namespace gazebo {
    class MagnetPlugin : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
            this->base_ = _parent->GetLink("base_link");

            this->update_connection_ = 
                event::Events::ConnectWorldUpdateBegin(
                    std::bind(&MagnetPlugin::OnUpdate, this));
        }

        void OnUpdate() {
            double distance = this->base_->WorldPose().Pos().Z()
                            - ZERO_DISTANCE + MAGNET_Z_POS;

            // Convert to mm
            distance *= 1000.0;

            // Apply a downwards force proportional to the distance apart
            double force = 
                MAGNET_FORCE / (1 + MAGNET_CONST * pow(distance, 5));

            if (distance < 0)
                force = 0;

            this->base_->SetForce(ignition::math::Vector3d(0, 0, -force));
        }
    private:
        physics::LinkPtr base_;
        event::ConnectionPtr update_connection_;
    };

    GZ_REGISTER_MODEL_PLUGIN(MagnetPlugin);
}
