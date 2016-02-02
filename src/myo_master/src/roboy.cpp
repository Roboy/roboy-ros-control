#include "roboy.hpp"

HardwareInterface::HardwareInterface()
{
    init_srv = nh.advertiseService("/roboy/initialize", &HardwareInterface::initializeService, this);
    controller_manager_client = nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
}

bool HardwareInterface::initializeService(common_utilities::Initialize::Request  &req,
                              common_utilities::Initialize::Response &res)
{
    // allocate corresponding control arrays (4 motors can be connected to each ganglion)
    while(flexray.checkNumberOfConnectedGanglions()>6){
        ROS_ERROR_THROTTLE(5,"Flexray interface says %d ganglions are connected, check cabels and power", flexray.checkNumberOfConnectedGanglions());
    }
    ROS_DEBUG("Flexray interface says %d ganglions are connected", flexray.checkNumberOfConnectedGanglions());
    
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

		uint ganglion = req.idList[i]/4;
		uint motor = req.idList[i]%4;
		switch(req.controlmode[i]){
			case 1: {
				ROS_INFO("%s position controller",motorname);
				flexray.initPositionControl(ganglion, motor);
				// connect and register the joint position interface
				hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(motorname), &cmd[i]);
				jnt_pos_interface.registerHandle(pos_handle);
				break;
			}
			case 2: {
				ROS_INFO("%s veclocity controller",motorname);
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
				common_utilities::ControllerState msg;
				msg.id = req.idList[i];
				msg.state = STATUS::UNDEFINED;
				res.states.push_back(msg);
				continue;
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

    ROS_DEBUG("Hardware interface initialized");
    ROS_DEBUG("Resources registered to this hardware interface:\n%s", str.c_str());
    ROS_DEBUG("Waiting for controller");
    
    ready = true;

	interface = new NCursesInterface;
	interface->MotorsInitialized();

    return true;
}

HardwareInterface::~HardwareInterface()
{
	delete interface;
}

void HardwareInterface::read()
{
    ROS_DEBUG("read");
#ifdef HARDWARE
    flexray.exchangeData();
#endif
    uint i = 0;
	interface->clearAll(4);
    for (uint ganglion=0;ganglion<flexray.numberOfGanglionsConnected;ganglion++){ 
        // four motors can be connected to each ganglion
        for (uint motor=0;motor<4;motor++){ 
            pos[i] = flexray.GanglionData[ganglion].muscleState[motor].actuatorPos*flexray.controlparams.radPerEncoderCount;
            vel[i] = flexray.GanglionData[ganglion].muscleState[motor].actuatorVel*flexray.controlparams.radPerEncoderCount;
            eff[i] = flexray.GanglionData[ganglion].muscleState[motor].tendonDisplacement; // dummy TODO: use polynomial approx
            interface->statusMotor(ganglion, motor, flexray.motorState[i]);
            i++;
        }
    }
}
void HardwareInterface::write()
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
