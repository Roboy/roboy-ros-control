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
            nh->shutdown();
            delete nh;
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

            nh = new ros::NodeHandle;

            tendon_sub = nh->subscribe("/visual/tendon", 1, &TendonVisualizer::RenderTendon, this);
            COM_sub = nh->subscribe("/visual/COM", 1, &TendonVisualizer::RenderCOM, this);

            visualizeTendon_sub = nh->subscribe("/visual/visualizeTendon", 1, &TendonVisualizer::VisualizeTendon, this);
            visualizeForce_sub = nh->subscribe("/visual/visualizeForce", 1, &TendonVisualizer::VisualizeForce, this);
            visualizeCOM_sub = nh->subscribe("/visual/visualizeCOM", 1, &TendonVisualizer::VisualizeCOM, this);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            update_connection = event::Events::ConnectRender(
                    boost::bind(&TendonVisualizer::UpdateChild, this));

            ROS_INFO("Tendonvisualizer initialized");
        }

        //////////////////////////////////////////////////////////////////////////////////
        // Update the visualizer
        void TendonVisualizer::UpdateChild() {
            ros::spinOnce();
        }

        //////////////////////////////////////////////////////////////////////////////////
        // VisualizeForceOnLink
        void TendonVisualizer::RenderTendon(const roboy_simulation::TendonConstPtr &msg) {
            if(visualizeTendon_flag) {
                visual->DeleteDynamicLine(tendon);
                tendon = visual->CreateDynamicLine(RENDERING_LINE_STRIP);
                for (uint i = 0; i < msg->viaPoints.size(); i++) {
                    // tendon viapoints
                    math::Vector3 vp = math::Vector3(msg->viaPoints[i].x, msg->viaPoints[i].y, msg->viaPoints[i].z);
                    tendon->AddPoint(vp);
                }
                tendon->setMaterial("Gazebo/Purple");
                tendon->setVisibilityFlags(GZ_VISIBILITY_GUI);
                visual->SetVisible(true);
            }
            if(visualizeForce_flag){
                visual->DeleteDynamicLine(force);
                force = visual->CreateDynamicLine(RENDERING_LINE_LIST);
                for (uint i = 0; i < msg->viaPoints.size(); i++) {
                    // tendon viapoints
                    math::Vector3 vp = math::Vector3(msg->viaPoints[i].x,msg->viaPoints[i].y,msg->viaPoints[i].z);
                    tendon->AddPoint(vp);
                    math::Vector3 f = math::Vector3(msg->force[i].x,msg->force[i].y,msg->force[i].z);
//                math::Vector3 force_normalized = f.Normalize();
                    force->AddPoint( vp + f);
                }
                tendon->setMaterial("Gazebo/Purple");
                tendon->setVisibilityFlags(GZ_VISIBILITY_GUI);
                visual->SetVisible(true);
            }
        }

        void TendonVisualizer::RenderCOM(const geometry_msgs::Vector3ConstPtr &msg){

        }

        void TendonVisualizer::VisualizeTendon(const std_msgs::BoolConstPtr &msg){
            visualizeTendon_flag = msg->data;
        }
        void TendonVisualizer::VisualizeForce(const std_msgs::BoolConstPtr &msg){
            visualizeForce_flag = msg->data;
        }
        void TendonVisualizer::VisualizeCOM(const std_msgs::BoolConstPtr &msg){
            visualizeCOM_flag = msg->data;
        }

        // Register this plugin within the simulator
        GZ_REGISTER_VISUAL_PLUGIN(TendonVisualizer)
    }
}