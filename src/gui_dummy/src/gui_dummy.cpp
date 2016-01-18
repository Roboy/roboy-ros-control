#include "gui_dummy.hpp"

GUI::GUI(){
    initializeService = nh.serviceClient<common_utilities::Initialize>("/roboy/initialize");
    
    statusResponse = nh.subscribe("/roboy/statusResponse", 1000, &GUI::statusCallback, this);
}

GUI::~GUI(){}

// publish functions
void GUI::initRequest(vector<signed char> motor){
    common_utilities::Initialize msg;

    msg.request.idList = motor;

    initializeService.call(msg);
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
//    char trajectorymotortopic[50];
//    trajectoryPublisher.resize(trajectoryPublisher.size());
//    for(uint i=0;i<trajectoryPublisher.size();i++){
//        sprintf(trajectorymotortopic, "trajectory_motor%d", msg.controllers[i]);
//        trajectoryPublisher[i] = nh.advertise<common_utilities::Trajectory>(trajectorymotortopic, 1);
//    }
}

void GUI::statusCallback(common_utilities::Status msg){
    
}
