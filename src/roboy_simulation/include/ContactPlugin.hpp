#pragma once
// ros
#include <ros/ros.h>
#include <std_msgs/Bool.h>
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
//std
#include <string>

using namespace gazebo;

class ContactPlugin : public SensorPlugin {

public:
    /// \brief Constructor.
    ContactPlugin();

    /// \brief Destructor.
    virtual ~ContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] sdf SDF element that describes the plugin.
    virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);


private:
    /// \brief Callback that receives the contact sensor's update signal.
    virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    event::ConnectionPtr updateConnection;

    ros::NodeHandle *nh;
    ros::Publisher contact_pub;
};
