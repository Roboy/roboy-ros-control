#include <CommunicationData.h>
#include "roboy.hpp"

Roboy::Roboy()
{
    init_sub = nh.subscribe("/roboy/initialize", 1, &Roboy::initializeControllers, this);
	record_sub = nh.subscribe("/roboy/record", 1, &Roboy::record, this);
	steer_recording_sub = nh.subscribe("/roboy/steer_record",1000, &Roboy::steer_record, this);
	recordResult_pub = nh.advertise<common_utilities::RecordResult>("/roboy/recordResult",1000);

	cmd = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
	pos = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
	vel = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
	eff = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
}

Roboy::~Roboy()
{
}

void Roboy::initializeControllers( const common_utilities::Initialize::ConstPtr& msg )
{
    initialized = false;
    while(flexray.checkNumberOfConnectedGanglions()>6){
        ROS_ERROR_THROTTLE(5,"Flexray interface says %d ganglions are connected, check cabels and power", flexray.checkNumberOfConnectedGanglions());
    }
    ROS_DEBUG("Flexray interface says %d ganglions are connected", flexray.checkNumberOfConnectedGanglions());

    vector<string> start_controllers;
    for (uint i=0; i<msg->controllers.size(); i++){
        string resource = msg->controllers[i].resource;
        uint ganglion = msg->controllers[i].ganglion;
        uint motor = msg->controllers[i].motor;

        // connect and register the joint state interface
        start_controllers.push_back(resource);
		hardware_interface::JointStateHandle state_handle(resource, &pos[msg->controllers[i].id], &vel[msg->controllers[i].id], &eff[msg->controllers[i].id]);
		jnt_state_interface.registerHandle(state_handle);

		switch((uint)msg->controllers[i].controlmode){
			case 1: {
				ROS_INFO("%s position controller ganglion %d motor %d", resource.c_str(), ganglion, motor);
				flexray.initPositionControl(ganglion, motor);
				// connect and register the joint position interface
				hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(resource), &cmd[msg->controllers[i].id]);
				jnt_pos_interface.registerHandle(pos_handle);
				break;
			}
			case 2: {
				ROS_INFO("%s velocity controller ganglion %d motor %d", resource.c_str(), ganglion, motor);
				flexray.initVelocityControl(ganglion, motor);
				// connect and register the joint position interface
				hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(resource), &cmd[msg->controllers[i].id]);
				jnt_vel_interface.registerHandle(vel_handle);
				break;
			}
			case 3: {
				ROS_INFO("%s force controller ganglion %d motor %d", resource.c_str(), ganglion, motor);
				flexray.initForceControl(ganglion, motor);
				// connect and register the joint position interface
				hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(resource), &cmd[msg->controllers[i].id]);
				jnt_eff_interface.registerHandle(eff_handle);
				break;
			}
			default:
				ROS_WARN("The requested controlMode is not available, choose [1]PositionController [2]VelocityController [3]ForceController");
				break;
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

    ROS_INFO("Resources registered to hardware interface:\n%s", str.c_str());
    if(!loadControllers(start_controllers))
        return;

    ROS_INFO("Starting controllers now...");
    if(!startControllers(start_controllers))
        ROS_WARN("could not start POSITION CONTROLLERS, try starting via /controller_manager/switch_controller service");

	initialized = true;
}

void Roboy::read()
{
    ROS_DEBUG("read");

    flexray.exchangeData();

    uint i = 0;
#ifdef HARDWARE
    for (uint ganglion=0;ganglion<flexray.numberOfGanglionsConnected;ganglion++){
#else
    for (uint ganglion=0;ganglion<NUMBER_OF_GANGLIONS;ganglion++){
#endif
        // four motors can be connected to each ganglion
        for (uint motor=0;motor<NUMBER_OF_JOINTS_PER_GANGLION;motor++){
            pos[i] = flexray.GanglionData[ganglion].muscleState[motor].actuatorPos*flexray.controlparams.radPerEncoderCount;
            vel[i] = flexray.GanglionData[ganglion].muscleState[motor].actuatorVel*flexray.controlparams.radPerEncoderCount;
			float polyPar[4];
			polyPar[0]=0; polyPar[1]=0.237536; polyPar[2]=-0.000032; polyPar[3]=0;
			float tendonDisplacement = flexray.GanglionData[ganglion].muscleState[motor].tendonDisplacement;
			eff[i] = polyPar[0] + polyPar[1] * tendonDisplacement + polyPar[2] * powf(tendonDisplacement, 2.0f) + polyPar[3] * powf(tendonDisplacement, 3.0f);

			i++;
        }
    }
}
void Roboy::write()
{
    ROS_DEBUG("write");
    uint i = 0;
    for (uint ganglion=0;ganglion<NUMBER_OF_GANGLIONS;ganglion++){
        // four motors can be connected to each ganglion
        for (uint motor=0;motor<NUMBER_OF_JOINTS_PER_GANGLION;motor++){
            if(ganglion<3)  // write to first commandframe
                flexray.commandframe0[ganglion].sp[motor] = cmd[i];
            else            // else write to second commandframe
                flexray.commandframe1[ganglion-3].sp[motor] = cmd[i];
            i++;
        }
    }

    flexray.updateCommandFrame();
    flexray.exchangeData();
}

void Roboy::main_loop(controller_manager::ControllerManager *ControllerManager)
{
    cm = ControllerManager;

    // Control loop
	ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    currentState = WaitForInitialize;

    while (ros::ok()){
		switch(currentState){
			case WaitForInitialize: {
                ROS_INFO_THROTTLE(5, "%s", state_strings[WaitForInitialize].c_str());
                if(!initialized) {
                    // idle
                    continue;
                }else {
                    // go to next state
                    prev_time = ros::Time::now();
                    break;
                }
			}
			case Controlloop: {
				ROS_INFO_THROTTLE(10, "%s", state_strings[Controlloop].c_str());
				const ros::Time time = ros::Time::now();
				const ros::Duration period = time - prev_time;

				read();
				write();

				prev_time = time;

				rate.sleep();
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

bool Roboy::loadControllers(vector<string> controllers){
	bool controller_loaded = true;
	for (auto controller : controllers) {
		if(!cm->loadController(controller)) {
			controller_loaded = false;
		}
	}
	return controller_loaded;
}

bool Roboy::unloadControllers(vector<string> controllers){
    bool controller_loaded = true;
    for (auto controller : controllers) {
        if(!cm->unloadController(controller)) {
            controller_loaded = false;
        }
    }
	return controller_loaded;
}

bool Roboy::startControllers(vector<string> controllers) {
    vector<string> stop_controllers;
    int strictness = 1; // best effort
    return cm->switchController(controllers, stop_controllers, strictness);
}

bool Roboy::stopControllers(vector<string> controllers) {
    vector<string> start_controllers;
    int strictness = 1; // best effort
    return cm->switchController(start_controllers, controllers, strictness);
}

ActionState Roboy::NextState(ActionState s)
{
	ActionState newstate;
	switch (s)
	{
		case WaitForInitialize:
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

void Roboy::record( const common_utilities::Record::ConstPtr &msg ) {
	currentState = Recording;
	std::vector<std::vector<float>> trajectories;
	recording = PLAY_TRAJECTORY;
	vector<int> controllers;
	vector<int> controlmode;
	for(auto controller:msg->controllers){
		controllers.push_back(controller.id);
		controlmode.push_back(controller.controlmode);
	}
	float averageSamplingTime = flexray.recordTrajectories(msg->sampleRate, trajectories, controllers, controlmode, &recording);
	common_utilities::RecordResult res;
	res.trajectories.resize(msg->controllers.size());
	for(uint m=0; m<msg->controllers.size(); m++){
		res.trajectories[m].id = msg->controllers[m].id;
		res.trajectories[m].waypoints = trajectories[msg->controllers[m].id];
		res.trajectories[m].samplerate = averageSamplingTime;
	}
	recordResult_pub.publish(res);
	currentState = Controlloop;
}

void Roboy::steer_record(const common_utilities::Steer::ConstPtr& msg){
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
