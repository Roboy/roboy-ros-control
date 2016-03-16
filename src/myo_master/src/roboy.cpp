#include "roboy.hpp"

HardwareInterface::HardwareInterface()
{
    init_srv = nh.advertiseService("/roboy/initialize", &HardwareInterface::initializeService, this);
	record_srv = nh.advertiseService("/roboy/record", &HardwareInterface::recordService, this);
	steer_recording_sub = nh.subscribe("/roboy/steer_recoding",1000, &HardwareInterface::steer_recording, this);
    cm_LoadController = nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    cm_ListController = nh.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
    cm_ListControllerTypes = nh.serviceClient<controller_manager_msgs::ListControllerTypes>("/controller_manager/list_contoller_types");
    cm_SwitchController = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

    cmd = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
    pos = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
    vel = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
    eff = new double[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
}

bool HardwareInterface::initializeService(common_utilities::Initialize::Request  &req,
                              common_utilities::Initialize::Response &res)
{
	ready = false;
    // allocate corresponding control arrays (4 motors can be connected to each ganglion)
    while(flexray.checkNumberOfConnectedGanglions()>6){
        ROS_ERROR_THROTTLE(5,"Flexray interface says %d ganglions are connected, check cabels and power", flexray.checkNumberOfConnectedGanglions());
    }
    ROS_DEBUG("Flexray interface says %d ganglions are connected", flexray.checkNumberOfConnectedGanglions());
    
    char motorname[20];

    for (uint i=0; i<req.idList.size(); i++){
		sprintf(motorname, "motor%d", req.idList[i]);

		// connect and register the joint state interface
		hardware_interface::JointStateHandle state_handle(motorname, &pos[i], &vel[i], &eff[i]);
		jnt_state_interface.registerHandle(state_handle);

		uint ganglion = req.idList[i]/4;
		uint motor = req.idList[i]%4;
		switch((uint)req.controlmode[i]){
			case 1: {
				ROS_INFO("%s position controller",motorname);
				flexray.initPositionControl(ganglion, motor);
				// connect and register the joint position interface
				hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(motorname), &cmd[i]);
				jnt_pos_interface.registerHandle(pos_handle);
				break;
			}
			case 2: {
				ROS_INFO("%s velocity controller",motorname);
				flexray.initVelocityControl(ganglion, motor);
				// connect and register the joint position interface
				hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(motorname), &cmd[i]);
				jnt_vel_interface.registerHandle(vel_handle);
				break;
			}
			case 3: {
				ROS_INFO("%s force controller",motorname);
				flexray.initForceControl(ganglion, motor);
				// connect and register the joint position interface
				hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(motorname), &cmd[i]);
				jnt_eff_interface.registerHandle(eff_handle);
				break;
			}
			default:
				ROS_WARN("The requested controlMode is not available, choose [1]PositionController [2]VelocityController [3]ForceController");
				common_utilities::ControllerState msg;
				msg.id = req.idList[i];
				msg.state = STATUS::UNDEFINED;
				res.states.push_back(msg);
				break;
		}

		if(flexray.motorState[req.idList[i]] == 1) { // only for ready motors
			common_utilities::ControllerState msg;
			msg.id = req.idList[i];
			msg.state = STATUS::INITIALIZED;
			res.states.push_back(msg);
		}else{
			common_utilities::ControllerState msg;
			msg.id = req.idList[i];
			msg.state = STATUS::UNDEFINED;
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
    ROS_INFO("Waiting for controller");
    
    ready = true;

    return true;
}

bool HardwareInterface::recordService(common_utilities::Record::Request &req,
									  common_utilities::Record::Response &res) {
	std::vector<std::vector<float>> trajectories;
	recording = PLAY_TRAJECTORY;
	flexray.recordTrajectories(req.samplingTime, trajectories, req.idList, req.controlmode, &recording);
	res.trajectories.resize(req.idList.size());
	for(uint m=0; m<req.idList.size(); m++){
		res.trajectories[m].waypoints = trajectories[req.idList[m]];
	}
	return true;
}

void HardwareInterface::steer_recording(const common_utilities::Steer::ConstPtr& msg){
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

HardwareInterface::~HardwareInterface()
{
}

void HardwareInterface::read()
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
            eff[i] = flexray.GanglionData[ganglion].muscleState[motor].tendonDisplacement; // dummy TODO: use polynomial approx
            i++;
        }
    }
}
void HardwareInterface::write()
{
    ROS_DEBUG("write");
    uint i = 0;
#ifdef HARDWARE
    for (uint ganglion=0;ganglion<flexray.numberOfGanglionsConnected;ganglion++){
#else
    for (uint ganglion=0;ganglion<NUMBER_OF_GANGLIONS;ganglion++){
#endif
        // four motors can be connected to each ganglion
        for (uint motor=0;motor<4;motor++){ 
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

Roboy::Roboy()
{
    cm = new controller_manager::ControllerManager(&hardwareInterface);
    emergencyStop_srv = nh.advertiseService("/roboy/emergencyStop", &Roboy::emergencyStopService, this);
}

void Roboy::main_loop()
{
    // Control loop
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10);

//	ros::CallbackQueue hw_queue;
//	hw_queue.addCallback(boost::shared_ptr<ros::CallbackInterface>nh.getCallbackQueue());
//	ros::AsyncSpinner hw_spinner(1, &hw_queue);

    ros::AsyncSpinner spinner(10); // 10 threads
    spinner.start();

    bool controller_loaded = false;

    while (ros::ok() && !emergencyStop)
    {
        if(hardwareInterface.ready){
            if(controller_loaded) {
                const ros::Time time = ros::Time::now();
                const ros::Duration period = time - prev_time;

                hardwareInterface.read();
                cm->update(time, period);
                hardwareInterface.write();

                rate.sleep();
            }else{
                ROS_INFO("loading position controller");
                controller_loaded = true;
                // load position controller
                vector<string> resources = hardwareInterface.jnt_pos_interface.getNames();
                for (auto resource : resources) {
                    if(!cm->loadController(resource)) {
                        string controllerType = resource, controllerOnParameterServer, controller;
                        controllerType.append("/type");
                        nh.getParam(controllerType,controllerOnParameterServer);
                        ROS_WARN("Unable to load PositionController for %s, because parameter server assigned %s to it", resource.c_str(),controllerOnParameterServer.c_str());
                        controller = "roboy_controller/PositionController";
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
//				controller_manager_msgs::SwitchController msg;
//				msg.request.start_controllers = resources;
//				msg.request.strictness = 1;
//				hardwareInterface.cm_SwitchController.call(msg);
				ROS_INFO("loading velocity controller");
                // load velocity controller
                resources = hardwareInterface.jnt_vel_interface.getNames();
                for (auto resource : resources) {
                    if(!cm->loadController(resource)) {
                        string controllerType = resource, controllerOnParameterServer, controller;
                        controllerType.append("/type");
                        nh.getParam(controllerType,controllerOnParameterServer);
                        ROS_WARN("Unable to load VelocityController for %s, because parameter server assigned %s to it", resource.c_str(),controllerOnParameterServer.c_str());
                        controller = "roboy_controller/VelocityController";
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
//				msg.request.start_controllers = resources;
//				msg.request.strictness = 2;
//				hardwareInterface.cm_SwitchController.call(msg);
                // load force controller
				ROS_INFO("loading force controller");
                resources = hardwareInterface.jnt_eff_interface.getNames();
                for(auto resource : resources) {
                    if(!cm->loadController(resource)) {
                        string controllerType = resource, controllerOnParameterServer, controller;
                        controllerType.append("/type");
                        nh.getParam(controllerType,controllerOnParameterServer);
                        ROS_WARN("Unable to load ForceController for %s, because parameter server assigned %s to it", resource.c_str(),controllerOnParameterServer.c_str());
                        controller = "roboy_controller/ForceController";
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
//				msg.request.start_controllers = resources;
//				msg.request.strictness = 2;
//				hardwareInterface.cm_SwitchController.call(msg);
                if(controller_loaded) {
                    ROS_INFO("roboy ready");
//                    hardwareInterface.interface.show();
                }else{
                    hardwareInterface.ready = false;
                }
            }
        }else{
            ROS_INFO_THROTTLE(1,"roboy not ready");
        }
    }
}

bool Roboy::emergencyStopService(common_utilities::EmergencyStop::Request &req,
                          common_utilities::EmergencyStop::Response &res){
    if(req.all){
        emergencyStop=true;
    }else{
        for(auto i:req.idList) {
            char controllername[50];
            snprintf(controllername,50,"motor%d",i);
            cm->unloadController(controllername);
        }
    }
}
