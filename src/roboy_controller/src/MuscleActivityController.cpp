#include "MuscleActivityController.hpp"

MuscleActivityController::MuscleActivityController() {

};

bool MuscleActivityController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
    // get joint name from the parameter server
    if (!n.getParam("joint_name", joint_name)) {
        ROS_ERROR("Could not find joint name");
        myStatus = ControllerState::UNDEFINED;
        return false;
    }
    n.getParam("id", statusMsg.id);
    ROS_INFO("MuscleActivityController %d for %s initialized", statusMsg.id, joint_name.c_str());
    joint = hw->getHandle(joint_name);  // throws on failure
    setPoint_sub = n.subscribe("/roboy/setPoint_" + joint_name, 1, &MuscleActivityController::newSetpoint, this);
    steer_sub = n.subscribe("/roboy/steer", 1000, &MuscleActivityController::steer, this);
    status_pub = n.advertise<common_utilities::ControllerState>("/roboy/status_" + joint_name, 1000);
    setPoint_pub = n.advertise<std_msgs::Float32>("/roboy/setPoint_" + joint_name + "/eff", 1000);
    myStatus = ControllerState::INITIALIZED;
    statusMsg.state = myStatus;
    status_pub.publish(statusMsg);
    return true;
}

void MuscleActivityController::update(const ros::Time &time, const ros::Duration &period) {
    double eff = joint.getEffort();
    eff_msg.data = eff;
    setPoint_pub.publish(eff_msg);
    if (steered == PLAY_TRAJECTORY) {
        joint.setCommand(setpoint);
    }
}

void MuscleActivityController::steer(const common_utilities::Steer::ConstPtr &msg) {
    switch (msg->steeringCommand) {
        case STOP_TRAJECTORY:
            dt = 0;
            steered = STOP_TRAJECTORY;
            myStatus = TRAJECTORY_READY;
            ROS_INFO("%s received steering STOP", joint_name.c_str());
            break;
        case PLAY_TRAJECTORY:
            steered = PLAY_TRAJECTORY;
            myStatus = TRAJECTORY_PLAYING;
            ROS_INFO("%s received steering PLAY", joint_name.c_str());
            break;
        case PAUSE_TRAJECTORY:
            if (steered == PAUSE_TRAJECTORY) {
                steered = PLAY_TRAJECTORY;
                myStatus = TRAJECTORY_PLAYING;
            } else {
                steered = PAUSE_TRAJECTORY;
                myStatus = TRAJECTORY_READY;
            }
    }
    statusMsg.state = myStatus;
    status_pub.publish(statusMsg);
}

void MuscleActivityController::newSetpoint( const std_msgs::Float32::ConstPtr& msg ){
    lock_guard<mutex> guard(mutex); // locks setpoint until the function returns
    setpoint = msg->data;
}

PLUGINLIB_EXPORT_CLASS(MuscleActivityController, controller_interface::ControllerBase);


