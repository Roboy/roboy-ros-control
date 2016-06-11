#include <CommunicationData.h>
#include "roboySim.hpp"

namespace gazebo_ros_control {

	RoboySim::RoboySim() {
		roboyStateMsg.setPoint.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION);
		roboyStateMsg.actuatorPos.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION);
		roboyStateMsg.actuatorVel.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION);
		roboyStateMsg.tendonDisplacement.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION);
		roboyStateMsg.actuatorCurrent.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION);

		cmd = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
		pos = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
		vel = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
		eff = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
	}

	RoboySim::~RoboySim() {
		delete cm;
	}

	bool RoboySim::initializeService(common_utilities::Initialize::Request &req,
									 common_utilities::Initialize::Response &res) {
		if (!unloadControllers(ControlMode::POSITION_CONTROL))
			return false;
		if (!unloadControllers(ControlMode::VELOCITY_CONTROL))
			return false;
		if (!unloadControllers(ControlMode::FORCE_CONTROL))
			return false;
		initialized = false;

		char motorname[20];

		for (uint i = 0; i < req.idList.size(); i++) {
			sprintf(motorname, "motor%d", req.idList[i]);

			// connect and register the joint state interface
			hardware_interface::JointStateHandle state_handle(motorname, &pos[req.idList[i]], &vel[req.idList[i]],
															  &eff[req.idList[i]]);
			jnt_state_interface.registerHandle(state_handle);

			uint ganglion = req.idList[i] / 4;
			uint motor = req.idList[i] % 4;
			switch ((uint) req.controlmode[i]) {
				case 1: {
					ROS_INFO("%s position controller ganglion %d motor %d", motorname, ganglion, motor);
					// connect and register the joint position interface
					hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(motorname),
															   &cmd[req.idList[i]]);
					jnt_pos_interface.registerHandle(pos_handle);
					break;
				}
				case 2: {
					ROS_INFO("%s velocity controller ganglion %d motor %d", motorname, ganglion, motor);
					// connect and register the joint position interface
					hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(motorname),
															   &cmd[req.idList[i]]);
					jnt_vel_interface.registerHandle(vel_handle);
					break;
				}
				case 3: {
					ROS_INFO("%s force controller ganglion %d motor %d", motorname, ganglion, motor);
					// connect and register the joint position interface
					hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(motorname),
															   &cmd[req.idList[i]]);
					jnt_eff_interface.registerHandle(eff_handle);
					break;
				}
				default:
					ROS_WARN(
							"The requested controlMode is not available, choose [1]PositionController [2]VelocityController [3]ForceController");
					common_utilities::ControllerState msg;
					msg.id = req.idList[i];
					msg.state = ControllerState::UNDEFINED;
					res.states.push_back(msg);
					break;
			}

			if (true) { // only for ready motors
				common_utilities::ControllerState msg;
				msg.id = req.idList[i];
				msg.state = ControllerState::INITIALIZED;
				res.states.push_back(msg);
			} else {
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
		for (uint i = 0; i < resources.size(); i++) {
			str.append(resources[i]);
			str.append(" ");
		}

		registerInterface(&jnt_vel_interface);
		resources = jnt_vel_interface.getNames();
		for (uint i = 0; i < resources.size(); i++) {
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

	void RoboySim::readSim(ros::Time time, ros::Duration period) {
		ROS_DEBUG("read simulation");

		uint i = 0;

		for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
			// four motors can be connected to each ganglion
			for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
				pos[i] = 0;
				vel[i] = 0;
				float polyPar[4];
				polyPar[0] = 0;
				polyPar[1] = 0.237536;
				polyPar[2] = -0.000032;
				polyPar[3] = 0;
				float tendonDisplacement = 0;
				eff[i] = 0;

				i++;
			}
		}
	}

	void RoboySim::writeSim(ros::Time time, ros::Duration period) {
		ROS_DEBUG("write simulation");
		uint i = 0;
		for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
			// four motors can be connected to each ganglion
			for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
//				if (ganglion < 3)  // write to first commandframe
////                flexray.commandframe0[ganglion].sp[motor] = cmd[i];
//					cout << "write " << cmd << endl;
//				else            // else write to second commandframe
////                flexray.commandframe1[ganglion-3].sp[motor] = cmd[i];
//					cout << "write " << cmd << endl;
				i++;
			}
		}
	}

	bool RoboySim::loadControllers(int controlmode) {
		vector<string> resources;
		string controller;
		switch (controlmode) {
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
			if (!cm->loadController(resource)) {
				string controllerType = resource, controllerOnParameterServer;
				controllerType.append("/type");
				nh.getParam(controllerType, controllerOnParameterServer);
				ROS_WARN("Unable to load controller for %s, because parameter server assigned %s to it",
						 resource.c_str(), controllerOnParameterServer.c_str());
				ROS_INFO("Changing parameter server for %s: %s --> %s ", resource.c_str(),
						 controllerOnParameterServer.c_str(), controller.c_str());
				nh.setParam(controllerType, controller);
				ROS_INFO("Trying to load controller");
				if (!cm->loadController(resource)) {
					ROS_ERROR("Could not load %s for %s", controller.c_str(), resource.c_str());
					controller_loaded = false;
					continue;
				}
				ROS_INFO("Success");
			}
		}
		return controller_loaded;
	}

	bool RoboySim::unloadControllers(int controlmode) {
		vector<string> resources;
		string controller;
		switch (controlmode) {
			case ControlMode::POSITION_CONTROL:
				resources = jnt_pos_interface.getNames();
				controller = "roboy_controller/PositionController";
				break;
			case ControlMode::VELOCITY_CONTROL:
				resources = jnt_vel_interface.getNames();
				controller = "roboy_controller/VelocityController";
				break;
			case ControlMode::FORCE_CONTROL:
				resources = jnt_eff_interface.getNames();
				controller = "roboy_controller/ForceController";
				break;
		}
		bool controller_loaded = true;
		if (!resources.empty())
			ROS_INFO("unloading controllers");
		for (auto resource : resources) {
			if (!cm->unloadController(resource)) {
				string controllerType = resource, controllerOnParameterServer;
				controllerType.append("/type");
				nh.getParam(controllerType, controllerOnParameterServer);
				ROS_WARN("Unable to load controller for %s, because parameter server assigned %s to it",
						 resource.c_str(), controllerOnParameterServer.c_str());
				ROS_INFO("Changing parameter server for %s: %s --> %s ", resource.c_str(),
						 controllerOnParameterServer.c_str(), controller.c_str());
				nh.setParam(controllerType, controller);
				ROS_INFO("Trying to load controller");
				if (!cm->loadController(resource)) {
					ROS_ERROR("Could not load %s for %s", controller.c_str(), resource.c_str());
					controller_loaded = false;
					continue;
				}
				ROS_INFO("Success");
			}
		}
		return controller_loaded;
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

	void RoboySim::steer_record(const common_utilities::Steer::ConstPtr &msg) {
		switch (msg->steeringCommand) {
			case STOP_TRAJECTORY:
				recording = STOP_TRAJECTORY;
				ROS_INFO("Received STOP recording");
				break;
			case PAUSE_TRAJECTORY:
				if (recording == PAUSE_TRAJECTORY) {
					recording = PLAY_TRAJECTORY;
					ROS_INFO("Received RESUME recording");
				} else {
					recording = PAUSE_TRAJECTORY;
					ROS_INFO("Received PAUSE recording");
				}
				break;
		}
	}

// Overloaded Gazebo entry point
	void RoboySim::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) {
		ROS_INFO_STREAM_NAMED("gazebo_ros_control", "Loading gazebo_ros_control plugin");

		// Save pointers to the model
		parent_model_ = parent;
		sdf_ = sdf;

		// Error message if the model couldn't be found
		if (!parent_model_) {
			ROS_ERROR_STREAM_NAMED("loadThread", "parent model is NULL");
			return;
		}

		// Check that ROS has been initialized
		if (!ros::isInitialized()) {
			ROS_FATAL_STREAM_NAMED("gazebo_ros_control",
								   "A ROS node for Gazebo has not been initialized, unable to load plugin. "
								   <<
								   "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}

		// Get namespace for nodehandle
		if (sdf_->HasElement("robotNamespace")) {
			robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
		}
		else {
			robot_namespace_ = parent_model_->GetName(); // default
		}

		// Get robot_description ROS param name
		if (sdf_->HasElement("robotParam")) {
			robot_description_ = sdf_->GetElement("robotParam")->Get<std::string>();
		}
		else {
			robot_description_ = "robot_description"; // default
		}

		// Get the Gazebo simulation period
		ros::Duration gazebo_period(parent_model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize());

		// Decide the plugin control period
		if (sdf_->HasElement("controlPeriod")) {
			control_period_ = ros::Duration(sdf_->Get<double>("controlPeriod"));

			// Check the period against the simulation period
			if (control_period_ < gazebo_period) {
				ROS_ERROR_STREAM_NAMED("gazebo_ros_control", "Desired controller update period (" << control_period_
															 << " s) is faster than the gazebo simulation period (" <<
															 gazebo_period << " s).");
			}
			else if (control_period_ > gazebo_period) {
				ROS_WARN_STREAM_NAMED("gazebo_ros_control", "Desired controller update period (" << control_period_
															<< " s) is slower than the gazebo simulation period (" <<
															gazebo_period << " s).");
			}
		}
		else {
			control_period_ = gazebo_period;
			ROS_DEBUG_STREAM_NAMED("gazebo_ros_control",
								   "Control period not found in URDF/SDF, defaulting to Gazebo period of "
								   << control_period_);
		}

		// Get parameters/settings for controllers from ROS param server
		nh = ros::NodeHandle(robot_namespace_);

		// Initialize the emergency stop code.
		e_stop_active_ = false;
		last_e_stop_active_ = false;
		if (sdf_->HasElement("eStopTopic")) {
			const std::string e_stop_topic = sdf_->GetElement("eStopTopic")->Get<std::string>();
			e_stop_sub_ = nh.subscribe(e_stop_topic, 1, &RoboySim::eStopCB, this);
		}

		ROS_INFO_NAMED("gazebo_ros_control", "Starting gazebo_ros_control plugin in namespace: %s",
					   robot_namespace_.c_str());

		// Read urdf from ros parameter server then
		// setup actuators and mechanism control node.
		// This call will block if ROS is not properly initialized.
		const std::string urdf_string = getURDF(robot_description_);
		if (!parseTransmissionsFromURDF(urdf_string)) {
			ROS_ERROR_NAMED("gazebo_ros_control",
							"Error parsing URDF in gazebo_ros_control plugin, plugin not active.\n");
			return;
		}

		urdf::Model urdf_model;
		const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

		init_srv = nh.advertiseService("/roboy/initialize", &RoboySim::initializeService, this);
		record_srv = nh.advertiseService("/roboy/record", &RoboySim::recordService, this);
		steer_recording_sub = nh.subscribe("/roboy/steer_record", 1000, &RoboySim::steer_record, this);
		roboy_pub = nh.advertise<common_utilities::RoboyState>("/roboy/state", 1000);

		ROS_INFO("Waiting for initialization from GUI");
		while (!initialized) {
			usleep(10000);
		}

		if (!initSim(robot_namespace_, nh, parent_model_, urdf_model_ptr, transmissions_)) {
			ROS_FATAL_NAMED("gazebo_ros_control", "Could not initialize robot simulation interface");
			return;
		}

		// Listen to the update event. This event is broadcast every simulation iteration.
		update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&RoboySim::Update, this));

		ROS_INFO_NAMED("gazebo_ros_control", "Loaded gazebo_ros_control.");
	}

	bool RoboySim::initSim(const std::string &robot_namespace,
						   ros::NodeHandle model_nh,
						   gazebo::physics::ModelPtr parent_model,
						   const urdf::Model *const urdf_model,
						   std::vector<transmission_interface::TransmissionInfo> transmissions) {

		ROS_INFO("initializing simulation");
		// getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
		// parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
		const ros::NodeHandle joint_limit_nh(model_nh);

		// Resize vectors to our DOF
		gazebo::physics::Joint_V joints = parent_model->GetJoints();
		n_dof_ = joints.size();
		joint_names_.resize(n_dof_);
		joint_types_.resize(n_dof_);
		joint_lower_limits_.resize(n_dof_);
		joint_upper_limits_.resize(n_dof_);
		joint_effort_limits_.resize(n_dof_);
		joint_control_methods_.resize(n_dof_);
		pid_controllers_.resize(n_dof_);

		// Initialize values
		for (unsigned int j = 0; j < n_dof_; j++) {
			// Get the gazebo joint that corresponds to the robot joint.
			gazebo::physics::JointPtr joint = joints[j];
			joint_names_[j] = joint->GetName();
			ROS_INFO_NAMED("initSim", "init joint: %s", joint_names_[j].c_str());
			if (!joint) {
				ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_[j]
								 << "\" which is not in the gazebo model.");
				return false;
			}
			sim_joints_.push_back(joint);

			// connect and register the joint state interface
			hardware_interface::JointStateHandle state_handle(joint_names_[j], &pos[j], &vel[j], &eff[j]);
			jnt_state_interface.registerHandle(state_handle);

			uint ganglion = j / 4;
			uint motor = j % 4;

			uint controlMode = POSITION;

			hardware_interface::JointHandle joint_handle(jnt_state_interface.getHandle(joint_names_[j]), &cmd[j]);

			switch ((uint) controlMode) {
				case 1: {
					ROS_INFO("%s position controller ganglion %d motor %d", joint_names_[j].c_str(), ganglion, motor);
					// connect and register the joint position interface
					jnt_pos_interface.registerHandle(joint_handle);
					break;
				}
				case 2: {
					ROS_INFO("%s velocity controller ganglion %d motor %d", joint_names_[j].c_str(), ganglion, motor);
					// connect and register the joint position interface
					jnt_vel_interface.registerHandle(joint_handle);
					break;
				}
				case 3: {
					ROS_INFO("%s force controller ganglion %d motor %d", joint_names_[j].c_str(), ganglion, motor);
					// connect and register the joint position interface
					jnt_eff_interface.registerHandle(joint_handle);
					break;
				}
				default:
					ROS_WARN(
							"The requested controlMode is not available, choose [1]PositionController [2]VelocityController [3]ForceController");
					break;
			}

			registerJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j],
								joint_limit_nh, urdf_model,
								&joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
								&joint_effort_limits_[j]);

			if (joint_control_methods_[j] != EFFORT) {
				// Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
				// joint->SetParam("vel") to control the joint.
				const ros::NodeHandle nh(model_nh, "/gazebo_ros_control/pid_gains/" +
												   joint_names_[j]);
				if (pid_controllers_[j].init(nh, true)) {
					switch (joint_control_methods_[j]) {
						case POSITION:
							joint_control_methods_[j] = POSITION_PID;
							break;
						case VELOCITY:
							joint_control_methods_[j] = VELOCITY_PID;
							break;
					}
				}
				else {
					// joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
					// going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
					// going to be called.
#if GAZEBO_MAJOR_VERSION > 2
					joint->SetParam("fmax", 0, joint_effort_limits_[j]);
#else
					joint->SetMaxForce(0, joint_effort_limits_[j]);
#endif
				}
			}
		}

		// Create the controller manager
		ROS_INFO_STREAM_NAMED("ros_control_plugin", "Loading controller_manager");
		cm = new controller_manager::ControllerManager(this, nh);

		// load the controllers
		if (!loadControllers(ControlMode::POSITION_CONTROL))
			return false;
		if (!loadControllers(ControlMode::VELOCITY_CONTROL))
			return false;
		if (!loadControllers(ControlMode::FORCE_CONTROL))
			return false;

		// Initialize the emergency stop code.
		e_stop_active_ = false;
		last_e_stop_active_ = false;

		return true;
	}

// Called by the world update start event
	void RoboySim::Update() {
		// Get the simulation time and period
		gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
		ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
		ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

		eStopActive(e_stop_active_);

		// Check if we should update the controllers
		if (sim_period >= control_period_) {
			// Store this simulation time
			last_update_sim_time_ros_ = sim_time_ros;

			// Update the robot simulation with the state of the gazebo model
			readSim(sim_time_ros, sim_period);

			// Compute the controller commands
			bool reset_ctrlrs;
			if (e_stop_active_) {
				reset_ctrlrs = false;
				last_e_stop_active_ = true;
			}
			else {
				if (last_e_stop_active_) {
					reset_ctrlrs = true;
					last_e_stop_active_ = false;
				}
				else {
					reset_ctrlrs = false;
				}
			}
			cm->update(sim_time_ros, sim_period, reset_ctrlrs);
		}

		// Update the gazebo model with the result of the controller
		// computation
		writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
		last_write_sim_time_ros_ = sim_time_ros;
	}

// Called on world reset
	void RoboySim::Reset() {
		// Reset timing variables to not pass negative update periods to controllers on world reset
		last_update_sim_time_ros_ = ros::Time();
		last_write_sim_time_ros_ = ros::Time();
	}

// Get the URDF XML from the parameter server
	std::string RoboySim::getURDF(std::string param_name) const {
		std::string urdf_string;

		// search and wait for robot_description on param server
		while (urdf_string.empty()) {
			std::string search_param_name;
			if (nh.searchParam(param_name, search_param_name)) {
				ROS_INFO_ONCE_NAMED("gazebo_ros_control", "gazebo_ros_control plugin is waiting for model"
						" URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

				nh.getParam(search_param_name, urdf_string);
			}
			else {
				ROS_INFO_ONCE_NAMED("gazebo_ros_control", "gazebo_ros_control plugin is waiting for model"
						" URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

				nh.getParam(param_name, urdf_string);
			}

			usleep(100000);
		}
		ROS_INFO_STREAM_NAMED("gazebo_ros_control",
							  "Recieved urdf " << param_name.c_str() << " from param server, parsing...");

		return urdf_string;
	}

// Get Transmissions from the URDF
	bool RoboySim::parseTransmissionsFromURDF(const std::string &urdf_string) {
		transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
		return true;
	}

// Emergency stop callback
	void RoboySim::eStopCB(const std_msgs::BoolConstPtr &e_stop_active) {
		e_stop_active_ = e_stop_active->data;
	}

	void RoboySim::registerJointLimits(const std::string &joint_name,
									   const hardware_interface::JointHandle &joint_handle,
									   const ControlMethod ctrl_method,
									   const ros::NodeHandle &joint_limit_nh,
									   const urdf::Model *const urdf_model,
									   int *const joint_type, double *const lower_limit,
									   double *const upper_limit, double *const effort_limit) {
		*joint_type = urdf::Joint::UNKNOWN;
		*lower_limit = -std::numeric_limits<double>::max();
		*upper_limit = std::numeric_limits<double>::max();
		*effort_limit = std::numeric_limits<double>::max();

		joint_limits_interface::JointLimits limits;
		bool has_limits = false;
		joint_limits_interface::SoftJointLimits soft_limits;
		bool has_soft_limits = false;

		if (urdf_model != NULL) {
			const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
			if (urdf_joint != NULL) {
				*joint_type = urdf_joint->type;
				// Get limits from the URDF file.
				if (joint_limits_interface::getJointLimits(urdf_joint, limits))
					has_limits = true;
				if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
					has_soft_limits = true;
			}
		}
		// Get limits from the parameter server.
		if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
			has_limits = true;

		if (!has_limits)
			return;

		if (*joint_type == urdf::Joint::UNKNOWN) {
			// Infer the joint type.

			if (limits.has_position_limits) {
				*joint_type = urdf::Joint::REVOLUTE;
			}
			else {
				if (limits.angle_wraparound)
					*joint_type = urdf::Joint::CONTINUOUS;
				else
					*joint_type = urdf::Joint::PRISMATIC;
			}
		}

		if (limits.has_position_limits) {
			*lower_limit = limits.min_position;
			*upper_limit = limits.max_position;
		}
		if (limits.has_effort_limits)
			*effort_limit = limits.max_effort;

		if (has_soft_limits) {
			switch (ctrl_method) {
				case EFFORT: {
					const joint_limits_interface::EffortJointSoftLimitsHandle
							limits_handle(joint_handle, limits, soft_limits);
					ej_limits_interface_.registerHandle(limits_handle);
				}
					break;
				case POSITION: {
					const joint_limits_interface::PositionJointSoftLimitsHandle
							limits_handle(joint_handle, limits, soft_limits);
					pj_limits_interface_.registerHandle(limits_handle);
				}
					break;
				case VELOCITY: {
					const joint_limits_interface::VelocityJointSoftLimitsHandle
							limits_handle(joint_handle, limits, soft_limits);
					vj_limits_interface_.registerHandle(limits_handle);
				}
					break;
			}
		}
		else {
			switch (ctrl_method) {
				case EFFORT: {
					const joint_limits_interface::EffortJointSaturationHandle
							sat_handle(joint_handle, limits);
					ej_sat_interface_.registerHandle(sat_handle);
				}
					break;
				case POSITION: {
					const joint_limits_interface::PositionJointSaturationHandle
							sat_handle(joint_handle, limits);
					pj_sat_interface_.registerHandle(sat_handle);
				}
					break;
				case VELOCITY: {
					const joint_limits_interface::VelocityJointSaturationHandle
							sat_handle(joint_handle, limits);
					vj_sat_interface_.registerHandle(sat_handle);
				}
					break;
			}
		}
	}
}

GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::RoboySim)

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::RoboySim, gazebo_ros_control::RobotHWSim)