#include "ContactPlugin.hpp"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

ContactPlugin::ContactPlugin() : SensorPlugin() {
    // start ros node
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "contact_sensor",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = new ros::NodeHandle;
}

ContactPlugin::~ContactPlugin() {
    delete nh;
}

void ContactPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get the parent sensor.
#if GAZEBO_MAJOR_VERSION < 7
    parentSensor = boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
#else
    this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
#endif

    // Make sure the parent sensor is valid.
    if (!parentSensor) {
        gzerr << "ContactPlugin requires a ContactSensor.\n";
        return;
    }

    // Connect to the sensor update event.
    updateConnection = parentSensor->ConnectUpdated(std::bind(&ContactPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    parentSensor->SetActive(true);

    contact_pub = nh->advertise<std_msgs::Bool>("/roboy/"+sdf->GetAttribute("name")->GetAsString(), 1);

    ROS_INFO_NAMED("contact_sensor","%s loaded", sdf->GetAttribute("name")->GetAsString().c_str());
}

void ContactPlugin::OnUpdate() {
    // Get all the contacts.
    std_msgs::Bool msg;
#if GAZEBO_MAJOR_VERSION < 7
    msg.data = parentSensor->GetContacts().contact_size();
#else
    msg.data = parentSensor->Contacts().contact_size();
#endif
    contact_pub.publish(msg);
    ros::spinOnce();
}