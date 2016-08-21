#pragma once

// gazebo
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include <gazebo_msgs/ModelStates.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

// ros
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "roboy_simulation/VisualizationControl.h"

// boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
// std
#include <sstream>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>

#endif

namespace gazebo {
    class GAZEBO_VISIBLE GazeboRoboyOverlay : public GUIPlugin{
    Q_OBJECT

    public:
        GazeboRoboyOverlay();
        virtual ~GazeboRoboyOverlay();

    protected slots:
        void showTendon();

    private:
        unsigned int counter;
        transport::NodePtr node;
        transport::PublisherPtr factoryPub;
        ros::NodeHandle *nh;
        ros::Publisher visualizeTendon_pub, visualizeForce_pub, visualizeCOM_pub;

        enum{
            Tendon,
            COM,
            Force
        }visualization;
    };
}
