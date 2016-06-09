#include <CommunicationData.h>
#include "roboySim.hpp"

RoboySim::RoboySim()
{
    cm = new controller_manager::ControllerManager(this);
    init_srv = nh.advertiseService("/roboy/initialize", &RoboySim::initializeService, this);
    record_srv = nh.advertiseService("/roboy/record", &RoboySim::recordService, this);
    steer_recording_sub = nh.subscribe("/roboy/steer_record",1000, &RoboySim::steer_record, this);
    cm_LoadController = nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    cm_ListController = nh.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
    cm_ListControllerTypes = nh.serviceClient<controller_manager_msgs::ListControllerTypes>("/controller_manager/list_contoller_types");
    cm_SwitchController = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    roboy_pub = nh.advertise<common_utilities::RoboyState>("/roboy/state",1000);
    roboyStateMsg.setPoint.resize(NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION);
    roboyStateMsg.actuatorPos.resize(NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION);
    roboyStateMsg.actuatorVel.resize(NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION);
    roboyStateMsg.tendonDisplacement.resize(NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION);
    roboyStateMsg.actuatorCurrent.resize(NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION);

    cmd = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
    pos = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
    vel = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
    eff = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
}

RoboySim::~RoboySim()
{
    delete cm;
}

bool RoboySim::initializeService(common_utilities::Initialize::Request  &req,
                              common_utilities::Initialize::Response &res)
{
    if(!unloadControllers(ControlMode::POSITION_CONTROL))
        return false;
    if(!unloadControllers(ControlMode::VELOCITY_CONTROL))
        return false;
    if(!unloadControllers(ControlMode::FORCE_CONTROL))
        return false;
    initialized = false;

    char motorname[20];

    for (uint i=0; i<req.idList.size(); i++){
        sprintf(motorname, "motor%d", req.idList[i]);

        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle(motorname, &pos[req.idList[i]], &vel[req.idList[i]], &eff[req.idList[i]]);
        jnt_state_interface.registerHandle(state_handle);

        uint ganglion = req.idList[i]/4;
        uint motor = req.idList[i]%4;
        switch((uint)req.controlmode[i]){
            case 1: {
                ROS_INFO("%s position controller ganglion %d motor %d",motorname, ganglion, motor);
                // connect and register the joint position interface
                hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(motorname), &cmd[req.idList[i]]);
                jnt_pos_interface.registerHandle(pos_handle);
                break;
            }
            case 2: {
                ROS_INFO("%s velocity controller ganglion %d motor %d",motorname, ganglion, motor);
                // connect and register the joint position interface
                hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(motorname), &cmd[req.idList[i]]);
                jnt_vel_interface.registerHandle(vel_handle);
                break;
            }
            case 3: {
                ROS_INFO("%s force controller ganglion %d motor %d",motorname, ganglion, motor);
                // connect and register the joint position interface
                hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(motorname), &cmd[req.idList[i]]);
                jnt_eff_interface.registerHandle(eff_handle);
                break;
            }
            default:
                ROS_WARN("The requested controlMode is not available, choose [1]PositionController [2]VelocityController [3]ForceController");
                common_utilities::ControllerState msg;
                msg.id = req.idList[i];
                msg.state = ControllerState::UNDEFINED;
                res.states.push_back(msg);
                break;
        }

        if(true) { // only for ready motors
            common_utilities::ControllerState msg;
            msg.id = req.idList[i];
            msg.state = ControllerState::INITIALIZED;
            res.states.push_back(msg);
        }else{
            common_utilities::ControllerState msg;
            msg.id = req.idList[i];
            msg.state = ControllerState::UNDEFINED;
            res.states.push_back(msg);
        }
    }

    registerInterface(&jnt_state_interface);
    string str;
    registerInterface(&jnt_pos_interface);
    vector<string> resources = jnt_pos_interface.getNames();
    for(uint i=0; i<resources.size();i++){
        str.append(resources[i]);
        str.append(" ");
    }

    registerInterface(&jnt_vel_interface);
    resources = jnt_vel_interface.getNames();
    for(uint i=0; i<resources.size();i++){
        str.append(resources[i]);
        str.append(" ");
    }

    registerInterface(&jnt_eff_interface);
    resources = jnt_eff_interface.getNames();
    for (uint i = 0; i < resources.size(); i++) {
        str.append(resources[i]);
        str.append(" ");
    }

    ROS_INFO("Hardware interface initialized");
    ROS_INFO("Resources registered to this hardware interface:\n%s", str.c_str());

    initialized = true;
    return true;
}

void RoboySim::readSim()
{
    ROS_DEBUG("read simulation");

    uint i = 0;

    for (uint ganglion=0;ganglion<NUMBER_OF_GANGLIONS;ganglion++){
        // four motors can be connected to each ganglion
        for (uint motor=0;motor<NUMBER_OF_JOINTS_PER_GANGLION;motor++){
            pos[i] = 0;
            vel[i] = 0;
            float polyPar[4];
            polyPar[0]=0; polyPar[1]=0.237536; polyPar[2]=-0.000032; polyPar[3]=0;
            float tendonDisplacement = 0;
            eff[i] = 0;

            i++;
        }
    }
}
void RoboySim::writeSim()
{
    ROS_DEBUG("write simulation");
    uint i = 0;
    for (uint ganglion=0;ganglion<NUMBER_OF_GANGLIONS;ganglion++){
        // four motors can be connected to each ganglion
        for (uint motor=0;motor<NUMBER_OF_JOINTS_PER_GANGLION;motor++){
            if(ganglion<3)  // write to first commandframe
//                flexray.commandframe0[ganglion].sp[motor] = cmd[i];
                cout << "write " << cmd << endl;
            else            // else write to second commandframe
//                flexray.commandframe1[ganglion-3].sp[motor] = cmd[i];
                cout << "write " << cmd << endl;
            i++;
        }
    }
}

void RoboySim::main_loop()
{
    // Control loop
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10);

    ros::AsyncSpinner spinner(10); // 10 threads
    spinner.start();

    currentState = WaitForInitialize;

    while (ros::ok())
    {
        switch(currentState){
            case WaitForInitialize: {
                ros::Duration d(1);
                while (!initialized) {
                    ROS_INFO_THROTTLE(5, "%s", state_strings[WaitForInitialize].c_str());
                    d.sleep();
                }
                break;
            }
            case LoadControllers: {
                ROS_INFO_THROTTLE(1,  "%s", state_strings[LoadControllers].c_str());
                if(!loadControllers(ControlMode::POSITION_CONTROL))
                    return;
                if(!loadControllers(ControlMode::VELOCITY_CONTROL))
                    return;
                if(!loadControllers(ControlMode::FORCE_CONTROL))
                    return;
                prev_time = ros::Time::now();
                break;
            }
            case Controlloop: {
                ROS_INFO_THROTTLE(10, "%s", state_strings[Controlloop].c_str());
                const ros::Time time = ros::Time::now();
                const ros::Duration period = time - prev_time;

                read();
                cm->update(time, period);
                write();

                prev_time = time;

                rate.sleep();
                break;
            }
            case PublishState: {
                ROS_INFO_THROTTLE(10, "%s", state_strings[PublishState].c_str());
                uint m = 0;
                for(uint ganglion=0;ganglion<NUMBER_OF_GANGLIONS;ganglion++){
                    for(uint motor=0;motor<NUMBER_OF_JOINTS_PER_GANGLION;motor++){
                        if(ganglion<3)
                            roboyStateMsg.setPoint[m] = 0;
                        else
                            roboyStateMsg.setPoint[m] = 0;

                        roboyStateMsg.actuatorPos[m] = 0;
                        roboyStateMsg.actuatorVel[m] = 0;
                        roboyStateMsg.tendonDisplacement[m] = 0;
                        roboyStateMsg.actuatorCurrent[m] = 0;
                        m++;
                    }
                }
                roboy_pub.publish(roboyStateMsg);
                break;
            }
            case Recording: {
                ROS_INFO_THROTTLE(5, "%s", state_strings[Recording].c_str());
                ros::Duration d(1);
                d.sleep();
                break;
            }
        }
        // get next state from state machine
        currentState = NextState(currentState);
    }
}

bool RoboySim::loadControllers(int controlmode){
    vector<string> resources;
    string controller;
    switch(controlmode){
        case ControlMode::POSITION_CONTROL:
            ROS_INFO("loading position controller");
            resources = jnt_pos_interface.getNames();
            controller = "roboy_controller/PositionController";
            break;
        case ControlMode::VELOCITY_CONTROL:
            ROS_INFO("loading velocity controller");
            resources = jnt_vel_interface.getNames();
            controller = "roboy_controller/VelocityController";
            break;
        case ControlMode::FORCE_CONTROL:
            ROS_INFO("loading force controller");
            resources = jnt_eff_interface.getNames();
            controller = "roboy_controller/ForceController";
            break;
    }
    bool controller_loaded = true;
    for (auto resource : resources) {
        if(!cm->loadController(resource)) {
            string controllerType = resource, controllerOnParameterServer;
            controllerType.append("/type");
            nh.getParam(controllerType,controllerOnParameterServer);
            ROS_WARN("Unable to load controller for %s, because parameter server assigned %s to it", resource.c_str(),controllerOnParameterServer.c_str());
            ROS_INFO("Changing parameter server for %s: %s --> %s ", resource.c_str(),controllerOnParameterServer.c_str(),controller.c_str());
            nh.setParam(controllerType,controller);
            ROS_INFO("Trying to load controller");
            if(!cm->loadController(resource)) {
                ROS_ERROR("Could not load %s for %s", controller.c_str(), resource.c_str());
                controller_loaded = false;
                continue;
            }
            ROS_INFO("Success");
        }
    }
    return controller_loaded;
}

bool RoboySim::unloadControllers(int controlmode){
    vector<string> resources;
    string controller;
    switch(controlmode){
        case ControlMode::POSITION_CONTROL:
            ROS_INFO("loading position controller");
            resources = jnt_pos_interface.getNames();
            controller = "roboy_controller/PositionController";
            break;
        case ControlMode::VELOCITY_CONTROL:
            ROS_INFO("loading velocity controller");
            resources = jnt_vel_interface.getNames();
            controller = "roboy_controller/VelocityController";
            break;
        case ControlMode::FORCE_CONTROL:
            ROS_INFO("loading force controller");
            resources = jnt_eff_interface.getNames();
            controller = "roboy_controller/ForceController";
            break;
    }
    bool controller_loaded = true;
    for (auto resource : resources) {
        if(!cm->unloadController(resource)) {
            string controllerType = resource, controllerOnParameterServer;
            controllerType.append("/type");
            nh.getParam(controllerType,controllerOnParameterServer);
            ROS_WARN("Unable to load controller for %s, because parameter server assigned %s to it", resource.c_str(),controllerOnParameterServer.c_str());
            ROS_INFO("Changing parameter server for %s: %s --> %s ", resource.c_str(),controllerOnParameterServer.c_str(),controller.c_str());
            nh.setParam(controllerType,controller);
            ROS_INFO("Trying to load controller");
            if(!cm->loadController(resource)) {
                ROS_ERROR("Could not load %s for %s", controller.c_str(), resource.c_str());
                controller_loaded = false;
                continue;
            }
            ROS_INFO("Success");
        }
    }
    return controller_loaded;
}

ActionState RoboySim::NextState(ActionState s)
{
    ActionState newstate;
    switch (s)
    {
        case WaitForInitialize:
            newstate = LoadControllers;
            break;
        case LoadControllers:
            newstate = Controlloop;
            break;
        case Controlloop:
            newstate = PublishState;
            break;
        case PublishState:
            newstate = Controlloop;
            break;
        case Recording:
            newstate = Recording;
            break;
    }
    return newstate;
}

bool RoboySim::recordService(common_utilities::Record::Request &req,
                          common_utilities::Record::Response &res) {
//    currentState = Recording;
//    std::vector<std::vector<float>> trajectories;
//    recording = PLAY_TRAJECTORY;
//    vector<int> controllers;
//    vector<int> controlmode;
//    for(auto controller:req.controllers){
//        controllers.push_back(controller.id);
//        controlmode.push_back(controller.controlmode);
//    }
//    float averageSamplingTime = flexray.recordTrajectories(req.sampleRate, trajectories, controllers, controlmode, &recording);
//    res.trajectories.resize(req.controllers.size());
//    for(uint m=0; m<req.controllers.size(); m++){
//        res.trajectories[m].id = req.controllers[m].id;
//        res.trajectories[m].waypoRoboySimints = trajectories[req.controllers[m].id];
//        res.trajectories[m].samplerate = averageSamplingTime;
//    }
//    currentState = Controlloop;
    return false;
}

void RoboySim::steer_record(const common_utilities::Steer::ConstPtr& msg){
    switch (msg->steeringCommand){
        case STOP_TRAJECTORY:
            recording = STOP_TRAJECTORY;
            ROS_INFO("Received STOP recording");
            break;
        case PAUSE_TRAJECTORY:
            if (recording==PAUSE_TRAJECTORY) {
                recording = PLAY_TRAJECTORY;
                ROS_INFO("Received RESUME recording");
            }else {
                recording = PAUSE_TRAJECTORY;
                ROS_INFO("Received PAUSE recording");
            }
            break;
    }
}
