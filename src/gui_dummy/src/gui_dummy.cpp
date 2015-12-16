#include "gui_dummy.hpp"

GUI::GUI(){
    initPublisher = nh.advertise<gui_dummy::InitializeRequest>("/roboy/initRequest", 1);
    
    initResponse = nh.subscribe("/roboy/initResponse", 1000, &GUI::initResponseCallback, this);
    
    statusResponse = nh.subscribe("/roboy/statusResponse", 1000, &GUI::statusCallback, this);
}

GUI::~GUI(){}

// publish functions
void GUI::initRequest(vector<unsigned char> motor){
    gui_dummy::InitializeRequest msg;
    
    msg.enable = motor;
    
    initPublisher.publish<gui_dummy::InitializeRequest>(msg);
    
    ros::spinOnce();
}

bool GUI::sendTrajectory(uint motor, uint32_t sampleRate, uint8_t controlMode, vector<float> setpoints){
    gui_dummy::Trajectory msg;
    msg.samplerate = sampleRate;
    msg.controlmode = controlMode;
    msg.waypoints = setpoints;
    if(trajectoryPublisher.size()<motor){
        trajectoryPublisher[motor].publish<gui_dummy::Trajectory>(msg);
        ros::spinOnce();
        return true;
    }
    return false;
}

// callback functions
void GUI::initResponseCallback(gui_dummy::InitializeResponse msg){
    char trajectorymotortopic[20];
    trajectoryPublisher.resize(msg.status.size());
    for(uint i=0; i<msg.status.size();i++){
        sprintf(trajectorymotortopic, "trajectory_motor%d", i);
        trajectoryPublisher[i] = nh.advertise<gui_dummy::Trajectory>(trajectorymotortopic, 1);
    }
}

void GUI::statusCallback(gui_dummy::Status msg){
    
}
