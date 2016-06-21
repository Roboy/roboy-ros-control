#include "TendonVisualizer.hpp"

namespace gazebo {
    namespace rendering {

        ////////////////////////////////////////////////////////////////////////////////
        // Constructor
        TendonVisualizer::TendonVisualizer() {

        }

        ////////////////////////////////////////////////////////////////////////////////
        // Destructor
        TendonVisualizer::~TendonVisualizer() {
            // Finalize the visualizer
            this->nh->shutdown();
            delete this->nh;
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Load the plugin
        void TendonVisualizer::Load(VisualPtr _parent, sdf::ElementPtr _sdf) {
            this->visual = _parent;
            // start ros node
            if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_visual",
                          ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
            }

            ROS_INFO("TENDON IS HERE");

            this->nh = new ros::NodeHandle;

            this->tendon_visualizer_sub = this->nh->subscribe("/visual/tendon", 1,
                                                              &TendonVisualizer::VisualizeTendonAndForce, this);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->update_connection = event::Events::ConnectRender(
                    boost::bind(&TendonVisualizer::UpdateChild, this));
        }

        //////////////////////////////////////////////////////////////////////////////////
        // Update the visualizer
        void TendonVisualizer::UpdateChild() {
            ros::spinOnce();
        }

        //////////////////////////////////////////////////////////////////////////////////
        // VisualizeForceOnLink
        void TendonVisualizer::VisualizeTendonAndForce(const roboy_simulation::TendonConstPtr &msg) {
            this->visual->DeleteDynamicLine(this->lines);
            this->lines = this->visual->CreateDynamicLine(RENDERING_LINE_STRIP);
            math::Pose worldPose = this->visual->GetWorldPose();
            for (uint i = 0; i < msg->viaPoints.size(); i++) {
                this->lines->AddPoint(
                        math::Vector3(
                                msg->viaPoints[i].x,
                                msg->viaPoints[i].y,
                                msg->viaPoints[i].z
                        )
                );
            }
            this->lines->setMaterial("Gazebo/Purple");
            this->lines->setVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual->SetVisible(true);
        }

        // Register this plugin within the simulator
        GZ_REGISTER_VISUAL_PLUGIN(TendonVisualizer)
    }
}