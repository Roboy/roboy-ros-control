#include "gui_dummy.hpp"

GUI::GUI(){
    initPublisher = nh.advertise<common_utilities::InitializeRequest>("/roboy/initRequest", 1);
    
    initResponse = nh.subscribe("/roboy/initResponse", 1000, &GUI::initResponseCallback, this);
    
    statusResponse = nh.subscribe("/roboy/statusResponse", 1000, &GUI::statusCallback, this);
}

GUI::~GUI(){}

// publish functions
void GUI::initRequest(vector<signed char> motor){
    common_utilities::InitializeRequest msg;
    
    msg.idList = motor;
    
    initPublisher.publish<common_utilities::InitializeRequest>(msg);
    
    ros::spinOnce();
}

bool GUI::sendTrajectory(uint motor, uint32_t sampleRate, uint8_t controlMode, vector<float> setpoints){
    common_utilities::Trajectory msg;
    msg.samplerate = sampleRate;
    msg.controlmode = controlMode;
    msg.waypoints = setpoints;
    if(trajectoryPublisher.size()<motor){
        trajectoryPublisher[motor].publish<common_utilities::Trajectory>(msg);
        ros::spinOnce();
        return true;
    }
    return false;
}

// callback functions
void GUI::initResponseCallback(common_utilities::InitializeResponse msg){
    char trajectorymotortopic[50];
    trajectoryPublisher.resize(trajectoryPublisher.size());
    for(uint i=0;i<trajectoryPublisher.size();i++){
        sprintf(trajectorymotortopic, "trajectory_motor%d", msg.controllers[i].id);
        trajectoryPublisher[i] = nh.advertise<common_utilities::Trajectory>(trajectorymotortopic, 1);
    }
}

void GUI::statusCallback(common_utilities::Status msg){
    
}
