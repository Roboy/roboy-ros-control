#pragma once
// common definitions
#include "CommonDefinitions.h"
// ros
#include <ros/ros.h>
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
//std
#include <string>
// messages
#include "roboy_simulation/ForceTorque.h"
#include <std_msgs/Int32.h>

using namespace gazebo;

class ForceTorquePlugin : public SensorPlugin {

public:
    /// \brief Constructor.
    ForceTorquePlugin();

    /// \brief Destructor.
    virtual ~ForceTorquePlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] sdf SDF element that describes the plugin.
    virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);


private:
    /// \brief Callback that receives the contact sensor's update signal.
    virtual void OnUpdate();

    void updateID(const std_msgs::Int32::ConstPtr &msg);

    /// \brief Pointer to the force torque sensor
    sensors::ForceTorqueSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    event::ConnectionPtr updateConnection;

    ros::NodeHandle *nh;
    ros::Publisher force_torque_pub;
    ros::Subscriber roboyID_sub;
    int roboyID;
    int leg;
};
