#include "TendonVisualizer.hpp"

namespace gazebo
{
    namespace rendering
    {

        ////////////////////////////////////////////////////////////////////////////////
        // Constructor
        TendonVisualizer::TendonVisualizer():
                line(NULL)
        {

        }

        ////////////////////////////////////////////////////////////////////////////////
        // Destructor
        TendonVisualizer::~TendonVisualizer()
        {
            // Finalize the visualizer
            this->rosnode_->shutdown();
            delete this->rosnode_;
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Load the plugin
        void TendonVisualizer::Load( VisualPtr _parent, sdf::ElementPtr _sdf )
        {
            this->visual_ = _parent;

            this->visual_namespace_ = "visual/";

            // start ros node
            if (!ros::isInitialized())
            {
                int argc = 0;
                char** argv = NULL;
                ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
            }

            this->rosnode_ = new ros::NodeHandle(this->visual_namespace_);
            this->force_sub_ = this->rosnode_->subscribe("/some_force", 1000, &TendonVisualizer::VisualizeForceOnLink, this);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->update_connection_ = event::Events::ConnectRender(
                    boost::bind(&TendonVisualizer::UpdateChild, this));
        }

        //////////////////////////////////////////////////////////////////////////////////
        // Update the visualizer
        void TendonVisualizer::UpdateChild()
        {
            ros::spinOnce();
            this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

            //TODO: Get the current link position
            math::Pose link_pose = this->visual_->GetPose();
            //TODO: Get the current end position
            math::Pose endpoint(link_pose.pos+math::Vector3(1,0,0),link_pose.rot);

            // Add two points to a connecting line strip from link_pose to endpoint
            this->line->AddPoint(
                    math::Vector3(
                            link_pose.pos.x,
                            link_pose.pos.y,
                            link_pose.pos.z
                    )
            );
            this->line->AddPoint(math::Vector3(endpoint.pos.x, endpoint.pos.y, endpoint.pos.z));
            // set the Material of the line, in this case to purple
            this->line->setMaterial("Gazebo/Purple");
            this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual_->SetVisible(true);
        }

        //////////////////////////////////////////////////////////////////////////////////
        // VisualizeForceOnLink
        void TendonVisualizer::VisualizeForceOnLink(const geometry_msgs::PointConstPtr &force_msg)
        {
            this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

            //TODO: Get the current link position
            math::Pose link_pose = this->visual_->GetPose();
            //TODO: Get the current end position
            math::Pose endpoint(link_pose.pos+math::Vector3(1,0,0),link_pose.rot);

            // Add two points to a connecting line strip from link_pose to endpoint
            this->line->AddPoint(
                    math::Vector3(
                            link_pose.pos.x,
                            link_pose.pos.y,
                            link_pose.pos.z
                    )
            );
            this->line->AddPoint(math::Vector3(endpoint.pos.x, endpoint.pos.y, endpoint.pos.z));
            // set the Material of the line, in this case to purple
            this->line->setMaterial("Gazebo/Purple");
            this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual_->SetVisible(true);
        }

        // Register this plugin within the simulator
        GZ_REGISTER_VISUAL_PLUGIN(TendonVisualizer)
    }
}