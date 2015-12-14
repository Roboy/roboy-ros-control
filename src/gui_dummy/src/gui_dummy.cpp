#include "gui_dummy.hpp"

GUI::GUI(){
    initPublisher = nh.advertise<gui_dummy::InitializeRequest>("init", 1);
    
    initResponse = nh.subscribe("initResponse", 1, &GUI::initResponseCB, this);
    
    statusResponse = nh.subscribe("statusResponse", 1, &GUI::initResponseCB, this);
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
void GUI::initResponseCB(gui_dummy::InitializeResponse msg){
    char trajectorymotortopic[20];
    trajectoryPublisher.resize(msg.status.size());
    for(uint i=0; i<msg.status.size();i++){
        sprintf(trajectorymotortopic, "trajectory_motor%d", i);
        trajectoryPublisher[i] = nh.advertise<gui_dummy::Trajectory>(trajectorymotortopic, 1);
    }
}

void GUI::statusCB(gui_dummy::Status msg){
    
}
