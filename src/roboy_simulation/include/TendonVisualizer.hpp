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
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelStates.h>

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

            DynamicLines *tendon;

            MovableText *force_text;

            /// \Subscribe to some force
            ros::Subscriber tendon_visualizer_sub;

            /// \brief Visualize the force
            void VisualizeTendonAndForce(const roboy_simulation::TendonConstPtr &msg);

            // Pointer to the update event connection
            event::ConnectionPtr update_connection;
        };
    }
}