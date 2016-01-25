#include <CommonDefinitions.h>
#include "gui_dummy.hpp"

GUI::GUI(){
    initialize_srv = nh.serviceClient<common_utilities::Initialize>("/roboy/initialize");

	// this is for asyncronous ros callbacks
	ros::AsyncSpinner spinner(1);
	spinner.start();
}

GUI::~GUI(){}

// publish functions
void GUI::initRequest(vector<signed char> motor){
    common_utilities::Initialize msg;

    msg.request.idList = motor;
    initialize_srv.call(msg);
    for(uint i=0; i<msg.response.states.size(); i++){
        if(msg.response.states[i].state==STATUS::INITIALIZED)
            trajectory_srvs[msg.response.states[i].id]=nh.serviceClient<common_utilities::Trajectory>("/roboy/trajectory");
    }
}

bool GUI::sendTrajectory(uint motor, uint32_t sampleRate, uint8_t controlMode, vector<float> setpoints){
    common_utilities::Trajectory msg;
    msg.request.samplerate = sampleRate;
    msg.request.controlmode = controlMode;
    msg.request.waypoints = setpoints;
    if(trajectory_srvs.size()<motor){
        trajectory_srvs[motor].call<common_utilities::Trajectory>(msg);
        return true;
    }
    return false;
}

