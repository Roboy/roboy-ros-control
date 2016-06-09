#include <CommunicationData.h>
#include "roboySim.hpp"
namespace gazebo_ros_control{

	RoboySim::RoboySim() {
		init_srv = nh.advertiseService("/roboy/initialize", &RoboySim::initializeService, this);
		record_srv = nh.advertiseService("/roboy/record", &RoboySim::recordService, this);
		steer_recording_sub = nh.subscribe("/roboy/steer_record", 1000, &RoboySim::steer_record, this);
		cm_LoadController = nh.serviceClient<controller_manager_msgs::LoadController>(
				"/controller_manager/load_controller");
		cm_ListController = nh.serviceClient<controller_manager_msgs::ListControllers>(
				"/controller_manager/list_controllers");
		cm_ListControllerTypes = nh.serviceClient<controller_manager_msgs::ListControllerTypes>(
				"/controller_manager/list_contoller_types");
		cm_SwitchController = nh.serviceClient<controller_manager_msgs::SwitchController>(
				"/controller_manager/switch_controller");
		roboy_pub = nh.advertise<common_utilities::RoboyState>("/roboy/state", 1000);
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
				if (ganglion < 3)  // write to first commandframe
//                flexray.commandframe0[ganglion].sp[motor] = cmd[i];
					cout << "write " << cmd << endl;
				else            // else write to second commandframe
//                flexray.commandframe1[ganglion-3].sp[motor] = cmd[i];
					cout << "write " << cmd << endl;
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
			if (!controller_manager_->loadController(resource)) {
				string controllerType = resource, controllerOnParameterServer;
				controllerType.append("/type");
				nh.getParam(controllerType, controllerOnParameterServer);
				ROS_WARN("Unable to load controller for %s, because parameter server assigned %s to it",
						 resource.c_str(), controllerOnParameterServer.c_str());
				ROS_INFO("Changing parameter server for %s: %s --> %s ", resource.c_str(),
						 controllerOnParameterServer.c_str(), controller.c_str());
				nh.setParam(controllerType, controller);
				ROS_INFO("Trying to load controller");
				if (!controller_manager_->loadController(resource)) {
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
			if (!controller_manager_->unloadController(resource)) {
				string controllerType = resource, controllerOnParameterServer;
				controllerType.append("/type");
				nh.getParam(controllerType, controllerOnParameterServer);
				ROS_WARN("Unable to load controller for %s, because parameter server assigned %s to it",
						 resource.c_str(), controllerOnParameterServer.c_str());
				ROS_INFO("Changing parameter server for %s: %s --> %s ", resource.c_str(),
						 controllerOnParameterServer.c_str(), controller.c_str());
				nh.setParam(controllerType, controller);
				ROS_INFO("Trying to load controller");
				if (!controller_manager_->loadController(resource)) {
					ROS_ERROR("Could not load %s for %s", controller.c_str(), resource.c_str());
					controller_loaded = false;
					continue;
				}
				ROS_INFO("Success");
			}
		}
		return controller_loaded;
	}

	ActionState RoboySim::NextState(ActionState s) {
		ActionState newstate;
		switch (s) {
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
	void RoboySim::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
	{
		ROS_INFO_STREAM_NAMED("gazebo_ros_control","Loading gazebo_ros_control plugin");


		// Save pointers to the model
		parent_model_ = parent;
		sdf_ = sdf;

		// Error message if the model couldn't be found
		if (!parent_model_)
		{
			ROS_ERROR_STREAM_NAMED("loadThread","parent model is NULL");
			return;
		}

		// Check that ROS has been initialized
		if(!ros::isInitialized())
		{
			ROS_FATAL_STREAM_NAMED("gazebo_ros_control","A ROS node for Gazebo has not been initialized, unable to load plugin. "
														<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}

		// Get namespace for nodehandle
		if(sdf_->HasElement("robotNamespace"))
		{
			robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
		}
		else
		{
			robot_namespace_ = parent_model_->GetName(); // default
		}

		// Get robot_description ROS param name
		if (sdf_->HasElement("robotParam"))
		{
			robot_description_ = sdf_->GetElement("robotParam")->Get<std::string>();
		}
		else
		{
			robot_description_ = "robot_description"; // default
		}

		// Get the robot simulation interface type
		if(sdf_->HasElement("robotSimType"))
		{
			robot_hw_sim_type_str_ = sdf_->Get<std::string>("robotSimType");
		}
		else
		{
			robot_hw_sim_type_str_ = "gazebo_ros_control/DefaultRobotHWSim";
			ROS_INFO_STREAM_NAMED("loadThread","Using default plugin for RobotHWSim (none specified in URDF/SDF)\""<<robot_hw_sim_type_str_<<"\"");
		}

		// Get the Gazebo simulation period
		ros::Duration gazebo_period(parent_model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize());

		// Decide the plugin control period
		if(sdf_->HasElement("controlPeriod"))
		{
			control_period_ = ros::Duration(sdf_->Get<double>("controlPeriod"));

			// Check the period against the simulation period
			if( control_period_ < gazebo_period )
			{
				ROS_ERROR_STREAM_NAMED("gazebo_ros_control","Desired controller update period ("<<control_period_
															<<" s) is faster than the gazebo simulation period ("<<gazebo_period<<" s).");
			}
			else if( control_period_ > gazebo_period )
			{
				ROS_WARN_STREAM_NAMED("gazebo_ros_control","Desired controller update period ("<<control_period_
														   <<" s) is slower than the gazebo simulation period ("<<gazebo_period<<" s).");
			}
		}
		else
		{
			control_period_ = gazebo_period;
			ROS_DEBUG_STREAM_NAMED("gazebo_ros_control","Control period not found in URDF/SDF, defaulting to Gazebo period of "
														<< control_period_);
		}

		// Get parameters/settings for controllers from ROS param server
		nh = ros::NodeHandle(robot_namespace_);

		// Initialize the emergency stop code.
		e_stop_active_ = false;
		last_e_stop_active_ = false;
		if (sdf_->HasElement("eStopTopic"))
		{
			const std::string e_stop_topic = sdf_->GetElement("eStopTopic")->Get<std::string>();
			e_stop_sub_ = nh.subscribe(e_stop_topic, 1, &RoboySim::eStopCB, this);
		}

		ROS_INFO_NAMED("gazebo_ros_control", "Starting gazebo_ros_control plugin in namespace: %s", robot_namespace_.c_str());

		// Read urdf from ros parameter server then
		// setup actuators and mechanism control node.
		// This call will block if ROS is not properly initialized.
		const std::string urdf_string = getURDF(robot_description_);
		if (!parseTransmissionsFromURDF(urdf_string))
		{
			ROS_ERROR_NAMED("gazebo_ros_control", "Error parsing URDF in gazebo_ros_control plugin, plugin not active.\n");
			return;
		}

		// Load the RobotHWSim abstraction to interface the controllers with the gazebo model
		try
		{
			robot_hw_sim_loader_.reset
					(new pluginlib::ClassLoader<gazebo_ros_control::RobotHWSim>
							 ("myo_master",
							  "gazebo_ros_control::RobotHWSim"));

			robot_hw_sim_ = robot_hw_sim_loader_->createInstance(robot_hw_sim_type_str_);
			urdf::Model urdf_model;
			const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

			if(!robot_hw_sim_->initSim(robot_namespace_, nh, parent_model_, urdf_model_ptr, transmissions_))
			{
				ROS_FATAL_NAMED("gazebo_ros_control","Could not initialize robot simulation interface");
				return;
			}

			// Create the controller manager
			ROS_INFO_STREAM_NAMED("ros_control_plugin","Loading controller_manager");
			controller_manager_.reset
					(new controller_manager::ControllerManager(robot_hw_sim_.get(), nh));

			// Listen to the update event. This event is broadcast every simulation iteration.
			update_connection_ =
					gazebo::event::Events::ConnectWorldUpdateBegin
							(boost::bind(&RoboySim::Update, this));

		}
		catch(pluginlib::LibraryLoadException &ex)
		{
			ROS_FATAL_STREAM_NAMED("gazebo_ros_control","Failed to create robot simulation interface loader: "<<ex.what());
		}

		ROS_INFO_NAMED("gazebo_ros_control", "Loaded gazebo_ros_control.");
	}

	bool RoboySim::initSim(
			const std::string& robot_namespace,
			ros::NodeHandle model_nh,
			gazebo::physics::ModelPtr parent_model,
			const urdf::Model *const urdf_model,
			std::vector<transmission_interface::TransmissionInfo> transmissions)
	{
		// getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
		// parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
		const ros::NodeHandle joint_limit_nh(model_nh);

		// Resize vectors to our DOF
		n_dof_ = transmissions.size();
		joint_names_.resize(n_dof_);
		joint_types_.resize(n_dof_);
		joint_lower_limits_.resize(n_dof_);
		joint_upper_limits_.resize(n_dof_);
		joint_effort_limits_.resize(n_dof_);
		joint_control_methods_.resize(n_dof_);
		pid_controllers_.resize(n_dof_);
		joint_position_.resize(n_dof_);
		joint_velocity_.resize(n_dof_);
		joint_effort_.resize(n_dof_);
		joint_effort_command_.resize(n_dof_);
		joint_position_command_.resize(n_dof_);
		joint_velocity_command_.resize(n_dof_);

		// Initialize values
		for(unsigned int j=0; j < n_dof_; j++)
		{
			// Check that this transmission has one joint
			if(transmissions[j].joints_.size() == 0)
			{
				ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
															 << " has no associated joints.");
				continue;
			}
			else if(transmissions[j].joints_.size() > 1)
			{
				ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
															 << " has more than one joint. Currently the default robot hardware simulation "
															 << " interface only supports one.");
				continue;
			}

			std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
			if (joint_interfaces.empty() &&
				!(transmissions[j].actuators_.empty()) &&
				!(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
			{
				// TODO: Deprecate HW interface specification in actuators in ROS J
				joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
				ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
															  transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
															  "The transmission will be properly loaded, but please update " <<
															  "your robot model to remain compatible with future versions of the plugin.");
			}
			if (joint_interfaces.empty())
			{
				ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
															  " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
															  "Not adding it to the robot hardware simulation.");
				continue;
			}
			else if (joint_interfaces.size() > 1)
			{
				ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
															  " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
															  "Currently the default robot hardware simulation interface only supports one. Using the first entry!");
				//continue;
			}

			// Add data from transmission
			joint_names_[j] = transmissions[j].joints_[0].name_;
			joint_position_[j] = 1.0;
			joint_velocity_[j] = 0.0;
			joint_effort_[j] = 1.0;  // N/m for continuous joints
			joint_effort_command_[j] = 0.0;
			joint_position_command_[j] = 0.0;
			joint_velocity_command_[j] = 0.0;

			const std::string& hardware_interface = joint_interfaces.front();

			// Debug
			ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim","Loading joint '" << joint_names_[j]
														  << "' of type '" << hardware_interface << "'");

			// Create joint state interface for all joints
			js_interface_.registerHandle(hardware_interface::JointStateHandle(
					joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

			// Decide what kind of command interface this actuator/joint has
			hardware_interface::JointHandle joint_handle;
			if(hardware_interface == "EffortJointInterface")
			{
				// Create effort joint interface
				joint_control_methods_[j] = EFFORT;
				joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
															   &joint_effort_command_[j]);
				ej_interface_.registerHandle(joint_handle);
			}
			else if(hardware_interface == "PositionJointInterface")
			{
				// Create position joint interface
				joint_control_methods_[j] = POSITION;
				joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
															   &joint_position_command_[j]);
				pj_interface_.registerHandle(joint_handle);
			}
			else if(hardware_interface == "VelocityJointInterface")
			{
				// Create velocity joint interface
				joint_control_methods_[j] = VELOCITY;
				joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
															   &joint_velocity_command_[j]);
				vj_interface_.registerHandle(joint_handle);
			}
			else
			{
				ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
															  << hardware_interface );
				return false;
			}

			// Get the gazebo joint that corresponds to the robot joint.
			//ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "Getting pointer to gazebo joint: "
			//  << joint_names_[j]);
			gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
			if (!joint)
			{
				ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_[j]
								 << "\" which is not in the gazebo model.");
				return false;
			}
			sim_joints_.push_back(joint);

			registerJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j],
								joint_limit_nh, urdf_model,
								&joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
								&joint_effort_limits_[j]);
			if (joint_control_methods_[j] != EFFORT)
			{
				// Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
				// joint->SetParam("vel") to control the joint.
				const ros::NodeHandle nh(model_nh, "/gazebo_ros_control/pid_gains/" +
												   joint_names_[j]);
				if (pid_controllers_[j].init(nh, true))
				{
					switch (joint_control_methods_[j])
					{
						case POSITION:
							joint_control_methods_[j] = POSITION_PID;
							break;
						case VELOCITY:
							joint_control_methods_[j] = VELOCITY_PID;
							break;
					}
				}
				else
				{
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

		// Register interfaces
		registerInterface(&js_interface_);
		registerInterface(&ej_interface_);
		registerInterface(&pj_interface_);
		registerInterface(&vj_interface_);

		// Initialize the emergency stop code.
		e_stop_active_ = false;
		last_e_stop_active_ = false;

		return true;
	}

// Called by the world update start event
	void RoboySim::Update()
	{
		// Get the simulation time and period
		gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
		ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
		ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

		robot_hw_sim_->eStopActive(e_stop_active_);

		// Check if we should update the controllers
		if(sim_period >= control_period_) {
			// Store this simulation time
			last_update_sim_time_ros_ = sim_time_ros;

			// Update the robot simulation with the state of the gazebo model
			robot_hw_sim_->readSim(sim_time_ros, sim_period);

			// Compute the controller commands
			bool reset_ctrlrs;
			if (e_stop_active_)
			{
				reset_ctrlrs = false;
				last_e_stop_active_ = true;
			}
			else
			{
				if (last_e_stop_active_)
				{
					reset_ctrlrs = true;
					last_e_stop_active_ = false;
				}
				else
				{
					reset_ctrlrs = false;
				}
			}
			controller_manager_->update(sim_time_ros, sim_period, reset_ctrlrs);
		}

		// Update the gazebo model with the result of the controller
		// computation
		robot_hw_sim_->writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
		last_write_sim_time_ros_ = sim_time_ros;
	}

// Called on world reset
	void RoboySim::Reset()
	{
		// Reset timing variables to not pass negative update periods to controllers on world reset
		last_update_sim_time_ros_ = ros::Time();
		last_write_sim_time_ros_ = ros::Time();
	}

// Get the URDF XML from the parameter server
	std::string RoboySim::getURDF(std::string param_name) const
	{
		std::string urdf_string;

		// search and wait for robot_description on param server
		while (urdf_string.empty())
		{
			std::string search_param_name;
			if (nh.searchParam(param_name, search_param_name))
			{
				ROS_INFO_ONCE_NAMED("gazebo_ros_control", "gazebo_ros_control plugin is waiting for model"
						" URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

				nh.getParam(search_param_name, urdf_string);
			}
			else
			{
				ROS_INFO_ONCE_NAMED("gazebo_ros_control", "gazebo_ros_control plugin is waiting for model"
						" URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

				nh.getParam(param_name, urdf_string);
			}

			usleep(100000);
		}
		ROS_INFO_STREAM_NAMED("gazebo_ros_control", "Recieved urdf " << param_name.c_str() << " from param server, parsing...");

		return urdf_string;
	}

// Get Transmissions from the URDF
	bool RoboySim::parseTransmissionsFromURDF(const std::string& urdf_string)
	{
		transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
		return true;
	}

// Emergency stop callback
	void RoboySim::eStopCB(const std_msgs::BoolConstPtr& e_stop_active)
	{
		e_stop_active_ = e_stop_active->data;
	}
}

GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::RoboySim)
PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::RoboySim, gazebo_ros_control::RobotHWSim)