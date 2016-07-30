#pragma once

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/MovableText.hh"
#include "gazebo/rendering/ArrowVisual.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/COMVisual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Bool.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "roboy_simulation/Tendon.h"

namespace gazebo
{
    namespace rendering
    {
        class TendonVisualizer : public VisualPlugin
        {
        public:
            /// \brief Constructor
            TendonVisualizer();

            /// \brief Destructor
            virtual ~TendonVisualizer();

            /// \brief Load the visual force plugin tags
            /// \param node XML config node
            void Load( VisualPtr _parent, sdf::ElementPtr _sdf );


        protected:
            /// \brief Update the visual plugin
            virtual void UpdateChild();


        private:
            /// \brief pointer to ros node
            ros::NodeHandle *nh;

            /// \brief store model name
            std::string model_name;

            /// \brief topic name
            std::string topic_name;

            // /// \brief The visual pointer used to visualize the force.
            VisualPtr visual;

            // /// \brief The scene pointer.
            ScenePtr scene;

            DynamicLines *tendon, *force;

            MovableText *force_text;

            COMVisualPtr COM;

            /// \Subscribe to some force
            ros::Subscriber tendon_sub, COM_sub, visualizeTendon_sub, visualizeForce_sub, visualizeCOM_sub;

            bool visualizeTendon_flag = false, visualizeForce_flag = false, visualizeCOM_flag = false;

            /// \brief Render the tendons
            void RenderTendon(const roboy_simulation::TendonConstPtr &msg);
            /// \brief Render the forces
            void RenderForce(const roboy_simulation::TendonConstPtr &msg);
            /// \brief Render the COM
            void RenderCOM(const geometry_msgs::Vector3ConstPtr &msg);

            /// \brief Render the tendons
            void VisualizeTendon(const std_msgs::BoolConstPtr &msg);
            /// \brief Render the forces
            void VisualizeForce(const std_msgs::BoolConstPtr &msg);
            /// \brief Render the COM
            void VisualizeCOM(const std_msgs::BoolConstPtr &msg);

            // Pointer to the update event connection
            event::ConnectionPtr update_connection;
        };
    }
}