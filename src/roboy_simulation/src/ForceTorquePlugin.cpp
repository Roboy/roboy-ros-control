#include "ForceTorquePlugin.hpp"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ForceTorquePlugin)

ForceTorquePlugin::ForceTorquePlugin() : SensorPlugin() {
    // start ros node
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "force_torque_sensor",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = new ros::NodeHandle;
}

ForceTorquePlugin::~ForceTorquePlugin() {
    delete nh;
}

void ForceTorquePlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get the parent sensor.
#if GAZEBO_MAJOR_VERSION < 7
    parentSensor = boost::dynamic_pointer_cast<sensors::ForceTorqueSensor>(sensor);
#else
    this->parentSensor = std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(sensor);
#endif

    // Make sure the parent sensor is valid.
    if (!parentSensor) {
        gzerr << "ForceTorquePlugin requires a ForceTorqueSensor.\n";
        return;
    }

    // Connect to the sensor update event.
    updateConnection = parentSensor->ConnectUpdated(std::bind(&ForceTorquePlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    parentSensor->SetActive(true);

    force_torque_pub = nh->advertise<roboy_simulation::ForceTorque>("/roboy/"+sdf->GetAttribute("name")->GetAsString(),
                                                                  1);
    if(sdf->GetAttribute("name")->GetAsString().find("left")!=std::string::npos)
        leg = LEG::LEFT;
    else if(sdf->GetAttribute("name")->GetAsString().find("right")!=std::string::npos)
        leg = LEG::RIGHT;

    ROS_INFO_NAMED("force_torque_sensor","%s loaded", sdf->GetAttribute("name")->GetAsString().c_str());
}

void ForceTorquePlugin::OnUpdate() {
    // Get all the contacts.
    roboy_simulation::ForceTorque msg;
    msg.leg = leg;
#if GAZEBO_MAJOR_VERSION < 7
    math::Vector3 force = parentSensor->Force();
    math::Vector3 torque = parentSensor->Torque();
    msg.force.x = force.X();
    msg.force.y = force.Y();
    msg.force.z = force.Z();
    msg.torque.x = torque.X();
    msg.torque.y = torque.Y();
    msg.torque.z = torque.Z();
#else
    ignition::math::Vector3d force = parentSensor->Force();
    ignition::math::Vector3d torque = parentSensor->Torque();
    msg.force.x = force.X();
    msg.force.y = force.Y();
    msg.force.z = force.Z();
    msg.torque.x = torque.X();
    msg.torque.y = torque.Y();
    msg.torque.z = torque.Z();
#endif
    force_torque_pub.publish(msg);
    ros::spinOnce();
}