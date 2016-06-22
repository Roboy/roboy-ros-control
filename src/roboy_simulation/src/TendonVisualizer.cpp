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
            visual = _parent;
            // start ros node
            if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_visual",
                          ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
            }

            ROS_INFO("TENDON IS HERE");

            nh = new ros::NodeHandle;

            tendon_visualizer_sub = nh->subscribe("/visual/tendon", 1,
                                                              &TendonVisualizer::VisualizeTendonAndForce, this);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            update_connection = event::Events::ConnectRender(
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
            visual->DeleteDynamicLine(tendon);
            tendon = visual->CreateDynamicLine(RENDERING_LINE_STRIP);
            for (uint i = 0; i < msg->viaPoints.size(); i++) {
                // tendon viapoints
                math::Vector3 vp = math::Vector3(msg->viaPoints[i].x,msg->viaPoints[i].y,msg->viaPoints[i].z);
                tendon->AddPoint(vp);
                // force vectors
//                this->force_vector->AddPoint(vp);
//                math::Vector3 f = math::Vector3(msg->force[i].x,msg->force[i].y,msg->force[i].z);
//                math::Vector3 force_normalized = f.Normalize();
//                this->force_vector->AddPoint( vp + force_normalized);
//                double norm = f.GetLength();
//                std::string text(norm);
//                this->force_text->SetText()
            }
            tendon->setMaterial("Gazebo/Purple");
            tendon->setVisibilityFlags(GZ_VISIBILITY_GUI);

//            this->force->setMaterial("Gazebo/Green");
//            this->force->setVisibilityFlags(GZ_VISIBILITY_GUI);

            visual->SetVisible(true);
        }

        // Register this plugin within the simulator
        GZ_REGISTER_VISUAL_PLUGIN(TendonVisualizer)
    }
}