#include "roboy_hardwareInterface.hpp"

Roboy::Roboy()
{
    init_srv = nh.advertiseService("/roboy/initialize", &Roboy::initializeService, this);
    
    controller_manager_client = nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
}

bool Roboy::initializeService(common_utilities::Initialize::Request  &req,
                              common_utilities::Initialize::Response &res)
{
    // allocate corresponding control arrays (4 motors can be connected to each ganglion)
    while(flexray.checkNumberOfConnectedGanglions()>6){
        ROS_ERROR("Flexray interface says %d ganglions are connected, check cabels and power", flexray.checkNumberOfConnectedGanglions());
        return false;
    }
    ROS_INFO("Flexray interface says %d ganglions are connected", flexray.checkNumberOfConnectedGanglions());
    
    cmd = new double[req.idList.size()];
    pos = new double[req.idList.size()];
    vel = new double[req.idList.size()];
    eff = new double[req.idList.size()];
    
    char motorname[10];
    
    for (uint i=0; i<req.idList.size(); i++){
        sprintf(motorname, "motor%d", req.idList[i]);
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle(motorname, &pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(state_handle);
        
        // connect and register the joint position interface
        hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(motorname), &cmd[i]);
        jnt_pos_interface.registerHandle(pos_handle);

        res.controllers.push_back(ControllerState::INITIALIZED);
    }
    
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
    
    ROS_INFO("Hardware interface initialized");
    string str;
    vector<string> resources = jnt_pos_interface.getNames();
    for(uint i=0; i<resources.size();i++){
        str.append(resources[i]);
        str.append(" ");
    }
    ROS_INFO("Resources registered to this hardware interface:\n%s", str.c_str());
    ROS_INFO("Waiting for controller");
    
    ready = true;
    return true;
}

Roboy::~Roboy()
{
    delete[] cmd;
    delete[] pos;
    delete[] vel;
    delete[] eff;
}

void Roboy::read()
{
    ROS_DEBUG("read");
#ifdef HARDWARE
    flexray.exchangeData();
#endif
    uint i = 0;
    for (uint ganglion=0;ganglion<flexray.numberOfGanglionsConnected;ganglion++){ 
        // four motors can be connected to each ganglion
        for (uint motor=0;motor<4;motor++){ 
            pos[i] = flexray.GanglionData[ganglion].muscleState[motor].actuatorPos*flexray.controlparams.radPerEncoderCount;
            vel[i] = flexray.GanglionData[ganglion].muscleState[motor].actuatorVel*flexray.controlparams.radPerEncoderCount;
            eff[i] = flexray.GanglionData[ganglion].muscleState[motor].tendonDisplacement; // dummy TODO: use polynomial approx
            i++;
        }
    }
}
void Roboy::write()
{
    ROS_DEBUG("write");
    uint i = 0;
    for (uint ganglion=0;ganglion<flexray.numberOfGanglionsConnected;ganglion++){ 
        // four motors can be connected to each ganglion
        for (uint motor=0;motor<4;motor++){ 
            if(ganglion<3)  // write to first commandframe
                flexray.commandframe0[ganglion].sp[motor] = cmd[i];
            else            // else write to second commandframe
                flexray.commandframe1[ganglion].sp[motor] = cmd[i];
            i++;
        }
    }
#ifdef HARDWARE
    flexray.updateCommandFrame();
    flexray.exchangeData();
#endif
}
