#include "roboySim.hpp"

namespace gazebo_ros_control {

    RoboySim::RoboySim() {
        init_sub = nh.subscribe("/roboy/initialize", 1, &RoboySim::initializeControllers, this);
        record_sub = nh.subscribe("/roboy/record", 1, &RoboySim::record, this);
        recordResult_pub = nh.advertise<common_utilities::RecordResult>("/roboy/recordResult", 1000);
        steer_recording_sub = nh.subscribe("/roboy/steer_record", 1000, &RoboySim::steer_record, this);

        tendon_visualizer_pub = nh.advertise<roboy_simulation::Tendon>("/visual/tendon", 1);

        cmd = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
        pos = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
        vel = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
        eff = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
    }

    RoboySim::~RoboySim() {
        delete cm;
        delete[] cmd, pos, vel, eff;
        update_thread->join();
        delete update_thread;
    }

    void RoboySim::initializeControllers(const common_utilities::Initialize::ConstPtr &msg) {
        initialized = false;
        sim_muscles.clear();

        // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
        // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
        const ros::NodeHandle joint_limit_nh(nh);

        // Resize vectors to our DOF
        gazebo::physics::Joint_V joints = parent_model->GetJoints();

        vector<string> start_controllers;
        for (uint i = 0; i < msg->controllers.size(); i++) {
            string resource = msg->controllers[i].resource;
            uint ganglion = msg->controllers[i].ganglion;
            uint motor = msg->controllers[i].motor;

            // connect and register the joint state interface
            start_controllers.push_back(resource);
            hardware_interface::JointStateHandle state_handle(resource, &pos[msg->controllers[i].id],
                                                              &vel[msg->controllers[i].id],
                                                              &eff[msg->controllers[i].id]);
            jnt_state_interface.registerHandle(state_handle);

            // Get the gazebo joint that corresponds to the robot joint.
            auto joint = std::find_if(joints.begin(), joints.end(),
                                      [&resource](gazebo::physics::JointPtr joint) -> bool {
                                          return resource.compare(joint->GetName()) == 0;
                                      });

            try {
                if (!*joint) {
                    ROS_ERROR_STREAM("You are trying to initialize a controller for " << resource <<
                                     " which is not in the gazebo model. Update your sdf/urdf files!");
                    continue;
                } else {
                    ROS_INFO("Loading Muscle Plugin");
                    sim_muscles.push_back(class_loader->createInstance("roboy_simulation::MusclePlugin"));
                    sim_muscles.back()->Init(myoMuscles[i]);
                }
                sim_joints.push_back(*joint);
            }
            catch (pluginlib::PluginlibException &ex) {
                //handle the class failing to load
                ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
            }

            hardware_interface::JointHandle joint_handle(jnt_state_interface.getHandle(resource),
                                                         &sim_muscles.back()->cmd);

            switch ((uint) msg->controllers[i].controlmode) {
                case 1: {
                    ROS_INFO("%s position controller ganglion %d motor %d", resource.c_str(), ganglion, motor);
                    // connect and register the joint position interface
                    jnt_pos_interface.registerHandle(joint_handle);
                    break;
                }
                case 2: {
                    ROS_INFO("%s velocity controller ganglion %d motor %d", resource.c_str(), ganglion, motor);
                    // connect and register the joint position interface
                    jnt_vel_interface.registerHandle(joint_handle);
                    break;
                }
                case 3: {
                    ROS_INFO("%s force controller ganglion %d motor %d", resource.c_str(), ganglion, motor);
                    // connect and register the joint position interface
                    jnt_eff_interface.registerHandle(joint_handle);
                    break;
                }
                default:
                    ROS_WARN(
                            "The requested controlMode is not available, choose [1]PositionController [2]VelocityController [3]ForceController");
                    break;
            }

            registerJointLimits(resource, joint_handle, joint_control_methods[i],
                                joint_limit_nh, &urdf_model,
                                &joint_types[i], &joint_lower_limits[i], &joint_upper_limits[i],
                                &joint_effort_limits[i]);

            if (joint_control_methods[i] != FORCE_CONTROL) {
                // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
                // joint->SetParam("vel") to control the joint.
                const ros::NodeHandle joint_nh(nh, "/roboy/pid_gains/" + resource);
                if (pid_controllers[i].init(joint_nh, true)) {
                    switch (joint_control_methods[i]) {
                        case POSITION_CONTROL:
                            joint_control_methods[i] = POSITION_CONTROL;
                            break;
                        case VELOCITY_CONTROL:
                            joint_control_methods[i] = VELOCITY_CONTROL;
                            break;
                    }
                }
                else {
                    // joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
                    // going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
                    // going to be called.
#if GAZEBO_MAJOR_VERSION > 2
                    (*joint)->SetParam("fmax", 0, joint_effort_limits[i]);
#else
                    (*joint)->SetMaxForce(0, joint_effort_limits[i]);
#endif
                }
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

        ROS_INFO("Resources registered to hardware interface:\n%s", str.c_str());
        loadControllers(start_controllers);

        ROS_INFO("Starting controllers now...");
        if (!startControllers(start_controllers))
            ROS_WARN(
                    "could not start POSITION CONTROLLERS, try starting via /controller_manager/switch_controller service");

        initialized = true;
    }

    void RoboySim::Load(gazebo::physics::ModelPtr parent_, sdf::ElementPtr sdf_) {
        ROS_INFO_STREAM_NAMED("gazebo_ros_control", "Loading gazebo_ros_control plugin");
        // Save pointers to the model
        parent_model = parent_;
        sdf = sdf_;

        // Error message if the model couldn't be found
        if (!parent_model) {
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
            robot_namespace = sdf_->GetElement("robotNamespace")->Get<std::string>();
        }
        else {
            robot_namespace = parent_model->GetName(); // default
        }

        // Get robot_description ROS param name
        if (sdf_->HasElement("robotParam")) {
            robot_description = sdf_->GetElement("robotParam")->Get<std::string>();
        }
        else {
            robot_description = "robot_description"; // default
        }

        // Get the Gazebo simulation period
        ros::Duration gazebo_period(parent_model->GetWorld()->GetPhysicsEngine()->GetMaxStepSize());

        // Decide the plugin control period
        control_period = ros::Duration(0.1);
        if (sdf_->HasElement("controlPeriod")) {
            control_period = ros::Duration(sdf_->Get<double>("controlPeriod"));

            ROS_INFO_STREAM_NAMED("gazebo_ros_control", "Desired controller update period: " << control_period );
            // Check the period against the simulation period
            if (control_period < gazebo_period) {
                ROS_ERROR_STREAM_NAMED("gazebo_ros_control", "Desired controller update period (" << control_period
                                                             << " s) is faster than the gazebo simulation period (" <<
                                                             gazebo_period << " s).");
            }
            else if (control_period > gazebo_period) {
                ROS_WARN_STREAM_NAMED("gazebo_ros_control", "Desired controller update period (" << control_period
                                                            << " s) is slower than the gazebo simulation period (" <<
                                                            gazebo_period << " s).");
            }
        }
        else {
            control_period = gazebo_period;
            ROS_DEBUG_STREAM_NAMED("gazebo_ros_control",
                                   "Control period not found in URDF/SDF, defaulting to Gazebo period of "
                                   << control_period);
        }

        // Get parameters/settings for controllers from ROS param server
        nh = ros::NodeHandle(robot_namespace);

        // Create the controller manager
        ROS_INFO_STREAM_NAMED("ros_control_plugin", "Loading controller_manager");
        cm = new controller_manager::ControllerManager(this, nh);

        // Initialize the emergency stop code.
        e_stop_active = false;
        last_e_stop_active = false;
        if (sdf_->HasElement("eStopTopic")) {
            const std::string e_stop_topic = sdf_->GetElement("eStopTopic")->Get<std::string>();
            e_stop_sub = nh.subscribe(e_stop_topic, 1, &RoboySim::eStopCB, this);
        }

        ROS_INFO_NAMED("gazebo_ros_control", "Starting gazebo_ros_control plugin in namespace: %s",
                       robot_namespace.c_str());

        // Read urdf from ros parameter server then
        // setup actuators and mechanism control node.
        // This call will block if ROS is not properly initialized.
        const std::string urdf_string = getURDF(robot_description);

        ROS_INFO_NAMED("gazebo_ros_control", "Parsing myoMuscles");
        if (!parseMyoMuscleSDF(sdf_->ToString(""), myoMuscles))
            ROS_WARN_NAMED("gazebo_ros_control", "ERROR parsing myoMuscles, check your sdf file.");
        numberOfMyoMuscles = myoMuscles.size();
        ROS_INFO("Found %d MyoMuscles in sdf file", numberOfMyoMuscles);

        joint_types.resize(numberOfMyoMuscles);
        joint_lower_limits.resize(numberOfMyoMuscles);
        joint_upper_limits.resize(numberOfMyoMuscles);
        joint_effort_limits.resize(numberOfMyoMuscles);
        joint_control_methods.resize(numberOfMyoMuscles);
        pid_controllers.resize(numberOfMyoMuscles);

        // class laoder for loading muscle plugins
        class_loader.reset(new pluginlib::ClassLoader<roboy_simulation::MusclePlugin>
                                   ("roboy_simulation",
                                    "roboy_simulation::MusclePlugin"));

        urdf_model.initString(urdf_string);

        // Listen to the update event. This event is broadcast every simulation iteration.
        update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&RoboySim::Update, this));

        ROS_INFO_NAMED("gazebo_ros_control", "Loaded gazebo_ros_control.");
    }

    void RoboySim::readSim(ros::Time time, ros::Duration period) {
        ROS_DEBUG("read simulation");
        // update muscle plugins
        force.clear();
        viaPointInGobalFrame.clear();
        for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
            for(auto link = sim_muscles[muscle]->linkPose.begin();
                link != sim_muscles[muscle]->linkPose.end(); ++link) {
                sim_muscles[muscle]->linkPose[link->first] = parent_model->GetLink(link->first)->GetWorldPose();
                    ROS_INFO_THROTTLE(1, "pose: %f %f %f", sim_muscles[muscle]->linkPose[link->first].pos.x,
                                      sim_muscles[muscle]->linkPose[link->first].pos.y,
                                      sim_muscles[muscle]->linkPose[link->first].pos.z);
            }
            sim_muscles[muscle]->Update(time, period, viaPointInGobalFrame, force);
            roboy_simulation::Tendon msg;
            for (uint i = 0; i < viaPointInGobalFrame.size(); i++) {
                geometry_msgs::Vector3 vp, f;
                vp.x = viaPointInGobalFrame[i].x;
                vp.y = viaPointInGobalFrame[i].y;
                vp.z = viaPointInGobalFrame[i].z;
                f.x = force[i].x;
                f.y = force[i].y;
                f.z = force[i].z;
                msg.viaPoints.push_back(vp);
                msg.force.push_back(f);
            }
            tendon_visualizer_pub.publish(msg);
        }
    }

    void RoboySim::writeSim(ros::Time time, ros::Duration period) {
        ROS_DEBUG("write simulation");
        // apply the calculated forces

        for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
            uint j = 0;
//            ROS_INFO_THROTTLE(1,"this muscle has %d links with viapoints", sim_muscles[muscle]->viaPoints.size());
            for(auto viaPoint = sim_muscles[muscle]->viaPoints.begin(); viaPoint != sim_muscles[muscle]->viaPoints.end(); ++viaPoint) {
                physics::LinkPtr link = parent_model->GetLink(viaPoint->first);
//                ROS_INFO_THROTTLE(1,"link %s, has %d viapoints", viaPoint->first.c_str(), viaPoint->second.size());

                for (uint i = 0; i < viaPoint->second.size(); i++) {

                    link->AddForceAtWorldPosition(force[j], viaPointInGobalFrame[j]);
//                    ROS_DEBUG("force on %s: %.4f %.4f %.4f at %.4f %.4f %.4f", viaPoint->first.c_str(),
//                                      force[j].x, force[j].y, force[j].z,
//                                      viaPointInGobalFrame[j].x, viaPointInGobalFrame[j].y, viaPointInGobalFrame[j].z);
                    j++;
                }
            }
        }
    }

    void RoboySim::Update() {
        // Get the simulation time and period
        gazebo::common::Time gz_time_now = parent_model->GetWorld()->GetSimTime();
        ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
        ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros;

        eStopActive(e_stop_active);

        // Check if we should update the controllers
        if (sim_period >= control_period) {
            // Store this simulation time
            last_update_sim_time_ros = sim_time_ros;

            // Update the robot simulation with the state of the gazebo model
            readSim(sim_time_ros, sim_period);

            // Compute the controller commands
            bool reset_ctrlrs;
            if (e_stop_active) {
                reset_ctrlrs = false;
                last_e_stop_active = true;
            }
            else {
                if (last_e_stop_active) {
                    reset_ctrlrs = true;
                    last_e_stop_active = false;
                }
                else {
                    reset_ctrlrs = false;
                }
            }
            cm->update(sim_time_ros, sim_period, reset_ctrlrs);
        }

        // Update the gazebo model with the result of the controller
        // computation
        writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros);
        last_write_sim_time_ros = sim_time_ros;
    }

    void RoboySim::Reset() {
        // Reset timing variables to not pass negative update periods to controllers on world reset
        last_update_sim_time_ros = ros::Time();
        last_write_sim_time_ros = ros::Time();
    }

    math::Vector3 RoboySim::calculateCOM(){
        physics::Link_V links = parent_model->GetLinks();
        double mass_total = 0;
        math::Vector3 COM(0,0,0);
        for(auto link:links){
            double m = link->GetInertial()->GetMass();
            mass_total += m;
            math::Pose p = link->GetWorldCoGPose();
            COM += p.pos*m;
        }
        return COM/mass_total;
    }

    bool RoboySim::loadControllers(vector<string> controllers) {
        bool controller_loaded = true;
        for (auto controller : controllers) {
            if (!cm->loadController(controller)) {
                controller_loaded = false;
            }
        }
        return controller_loaded;
    }

    bool RoboySim::unloadControllers(vector<string> controllers) {
        bool controller_loaded = true;
        for (auto controller : controllers) {
            if (!cm->unloadController(controller)) {
                controller_loaded = false;
            }
        }
        return controller_loaded;
    }

    bool RoboySim::startControllers(vector<string> controllers) {
        vector<string> stop_controllers;
        int strictness = 1; // best effort
        return cm->switchController(controllers, stop_controllers, strictness);
    }

    bool RoboySim::stopControllers(vector<string> controllers) {
        vector<string> start_controllers;
        int strictness = 1; // best effort
        return cm->switchController(start_controllers, controllers, strictness);
    }

    void RoboySim::record(const common_utilities::Record::ConstPtr &msg) {
        std::vector<std::vector<float>> trajectories;
        recording = PLAY_TRAJECTORY;
        vector<int> controllers;
        vector<int> controlmode;
        for (auto controller:msg->controllers) {
            controllers.push_back(controller.id);
            controlmode.push_back(controller.controlmode);
        }
//		float averageSamplingTime = flexray.recordTrajectories(msg->sampleRate, trajectories, controllers, controlmode, &recording);
        common_utilities::RecordResult res;
        res.trajectories.resize(msg->controllers.size());
        for (uint m = 0; m < msg->controllers.size(); m++) {
            res.trajectories[m].id = msg->controllers[m].id;
            res.trajectories[m].waypoints = trajectories[msg->controllers[m].id];
//			res.trajectories[m].samplerate = averageSamplingTime;
        }
        recordResult_pub.publish(res);
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
                        " URDF in parameter [%s] on the ROS param server.", robot_description.c_str());

                nh.getParam(param_name, urdf_string);
            }

            usleep(100000);
        }
        ROS_INFO_STREAM_NAMED("gazebo_ros_control",
                              "Recieved urdf " << param_name.c_str() << " from param server, parsing...");

        return urdf_string;
    }

    void RoboySim::eStopCB(const std_msgs::BoolConstPtr &e_stop_active) {

    }

    void RoboySim::registerJointLimits(const std::string &joint_name,
                                       const hardware_interface::JointHandle &joint_handle,
                                       const int ctrl_method,
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
                case FORCE_CONTROL: {
                    const joint_limits_interface::EffortJointSoftLimitsHandle
                            limits_handle(joint_handle, limits, soft_limits);
                    ej_limits_interface.registerHandle(limits_handle);
                }
                    break;
                case POSITION_CONTROL: {
                    const joint_limits_interface::PositionJointSoftLimitsHandle
                            limits_handle(joint_handle, limits, soft_limits);
                    pj_limits_interface.registerHandle(limits_handle);
                }
                    break;
                case VELOCITY_CONTROL: {
                    const joint_limits_interface::VelocityJointSoftLimitsHandle
                            limits_handle(joint_handle, limits, soft_limits);
                    vj_limits_interface.registerHandle(limits_handle);
                    break;
                }
            }
        }
        else {
            switch (ctrl_method) {
                case FORCE_CONTROL: {
                    const joint_limits_interface::EffortJointSaturationHandle
                            sat_handle(joint_handle, limits);
                    ej_sat_interface.registerHandle(sat_handle);
                }
                    break;
                case POSITION_CONTROL: {
                    const joint_limits_interface::PositionJointSaturationHandle
                            sat_handle(joint_handle, limits);
                    pj_sat_interface.registerHandle(sat_handle);
                }
                    break;
                case VELOCITY_CONTROL: {
                    const joint_limits_interface::VelocityJointSaturationHandle
                            sat_handle(joint_handle, limits);
                    vj_sat_interface.registerHandle(sat_handle);
                    break;
                }
            }
        }
    }

    bool RoboySim::parseMyoMuscleSDF(const string &sdf, vector<roboy_simulation::MyoMuscleInfo> &myoMuscles) {
        // initialize TiXmlDocument doc with a string
        TiXmlDocument doc;
        if (!doc.Parse(sdf.c_str()) && doc.Error()) {
            ROS_ERROR("Can't parse MyoMuscles. Invalid robot description.");
            return false;
        }

        // Find joints in transmission tags
        TiXmlElement *root = doc.RootElement();

        // Constructs the myoMuscles by parsing custom xml.
        TiXmlElement *myoMuscle_it = NULL;
        for (myoMuscle_it = root->FirstChildElement("myoMuscle"); myoMuscle_it;
             myoMuscle_it = myoMuscle_it->NextSiblingElement("myoMuscle")) {
            roboy_simulation::MyoMuscleInfo myoMuscle;
            if (myoMuscle_it->Attribute("name")) {
                myoMuscle.name = myoMuscle_it->Attribute("name");
                // myoMuscle joint acting on
                TiXmlElement *link_child_it = NULL;
                for (link_child_it = myoMuscle_it->FirstChildElement("link"); link_child_it;
                     link_child_it = link_child_it->NextSiblingElement("link")) {
                    string linkname = link_child_it->Attribute("name");
                    if (!linkname.empty()) {
                        TiXmlElement *viaPoint_child_it = NULL;
                        vector<math::Vector3> viaPoints;
                        for (viaPoint_child_it = link_child_it->FirstChildElement("viaPoint"); viaPoint_child_it;
                             viaPoint_child_it = viaPoint_child_it->NextSiblingElement("viaPoint")) {
                            float x, y, z;
                            if (sscanf(viaPoint_child_it->GetText(), "%f %f %f", &x, &y, &z) != 3) {
                                ROS_ERROR_STREAM_NAMED("parser", "error reading [via point] (x y z)");
                                return false;
                            } else {
                                viaPoints.push_back(math::Vector3(x, y, z));
                            }
                        }
                        myoMuscle.viaPoints[linkname] = viaPoints;
                        if (myoMuscle.viaPoints.empty()) {
                            ROS_ERROR_STREAM_NAMED("parser", "No viaPoint element found in myoMuscle '"
                                                             << myoMuscle.name << "' link element.");
                            return false;
                        }else{
                            for (auto viaPoint : myoMuscle.viaPoints[linkname]) {
                                ROS_INFO("%s viaPoint: %lf %lf %lf", linkname.c_str(), viaPoint.x, viaPoint.y, viaPoint.z);
                            }
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No link name attribute specified for myoMuscle'"
                                                         << myoMuscle.name << "'.");
                        continue;
                    }
                }

                TiXmlElement *motor_child = myoMuscle_it->FirstChildElement("motor");
                if (motor_child) {
                    // bemf_constant
                    TiXmlElement *bemf_constant_child = motor_child->FirstChildElement("bemf_constant");
                    if (bemf_constant_child) {
                        if (sscanf(bemf_constant_child->GetText(), "%lf", &myoMuscle.motor.BEMFConst) != 1) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading bemf_constant constant");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No bemf_constant element found in myoMuscle '"
                                                         << myoMuscle.name << "' motor element.");
                        return false;
                    }
                    // torque_constant
                    TiXmlElement *torque_constant_child = motor_child->FirstChildElement("torque_constant");
                    if (torque_constant_child) {
                        if (sscanf(torque_constant_child->GetText(), "%lf", &myoMuscle.motor.torqueConst) != 1) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading torque_constant constant");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No torque_constant element found in myoMuscle '"
                                                         << myoMuscle.name << "' motor element.");
                        return false;
                    }
                    // inductance
                    TiXmlElement *inductance_child = motor_child->FirstChildElement("inductance");
                    if (inductance_child) {
                        if (sscanf(inductance_child->GetText(), "%lf", &myoMuscle.motor.inductance) != 1) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading inductance constant");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No inductance element found in myoMuscle '"
                                                         << myoMuscle.name << "' motor element.");
                        return false;
                    }
                    // resistance
                    TiXmlElement *resistance_child = motor_child->FirstChildElement("resistance");
                    if (resistance_child) {
                        if (sscanf(resistance_child->GetText(), "%lf", &myoMuscle.motor.resistance) != 1) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading resistance constant");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No resistance element found in myoMuscle '"
                                                         << myoMuscle.name << "' motor element.");
                        return false;
                    }
                    // inertiaMoment
                    TiXmlElement *inertiaMoment_child = motor_child->FirstChildElement("inertiaMoment");
                    if (inertiaMoment_child) {
                        if (sscanf(inertiaMoment_child->GetText(), "%lf", &myoMuscle.motor.inertiaMoment) != 1) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading inertiaMoment constant");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No inertiaMoment element found in myoMuscle '"
                                                         << myoMuscle.name << "' motor element.");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No motor element found in myoMuscle '" << myoMuscle.name << "'.");
                    return false;
                }

                TiXmlElement *gear_child = myoMuscle_it->FirstChildElement("gear");
                if (gear_child) {
                    // ratio
                    TiXmlElement *ratio_child = gear_child->FirstChildElement("ratio");
                    if (ratio_child) {
                        if (sscanf(ratio_child->GetText(), "%lf", &myoMuscle.gear.ratio) != 1) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading ratio constant");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No ratio element found in myoMuscle '"
                                                         << myoMuscle.name << "' gear element.");
                        return false;
                    }
                    // ratio
                    TiXmlElement *efficiency_child = gear_child->FirstChildElement("efficiency");
                    if (efficiency_child) {
                        if (sscanf(efficiency_child->GetText(), "%lf", &myoMuscle.gear.efficiency) != 1) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading efficiency constant");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No efficiency element found in myoMuscle '"
                                                         << myoMuscle.name << "' gear element.");
                        return false;
                    }
                    // inertiaMoment
                    TiXmlElement *inertiaMoment_child = gear_child->FirstChildElement("inertiaMoment");
                    if (inertiaMoment_child) {
                        if (sscanf(inertiaMoment_child->GetText(), "%lf", &myoMuscle.gear.inertiaMoment) != 1) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading inertiaMoment constant");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No inertiaMoment element found in myoMuscle '"
                                                         << myoMuscle.name << "' gear element.");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No gear element found in myoMuscle '" << myoMuscle.name << "'.");
                    return false;
                }

                TiXmlElement *spindle_child = myoMuscle_it->FirstChildElement("spindle");
                if (spindle_child) {
                    // radius
                    TiXmlElement *radius_child = spindle_child->FirstChildElement("radius");
                    if (radius_child) {
                        if (sscanf(radius_child->GetText(), "%lf", &myoMuscle.spindle.radius) != 1) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading radius constant");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No radius element found in myoMuscle '"
                                                         << myoMuscle.name << "' spindle element.");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser",
                                           "No spindle element found in myoMuscle '" << myoMuscle.name << "'.");
                    return false;
                }

                TiXmlElement *SEE_child = myoMuscle_it->FirstChildElement("SEE");
                if (SEE_child) {
                    // stiffness
                    TiXmlElement *stiffness_child = SEE_child->FirstChildElement("stiffness");
                    if (stiffness_child) {
                        if (sscanf(stiffness_child->GetText(), "%lf", &myoMuscle.see.stiffness) != 1) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading radius constant");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No stiffness element found in myoMuscle '"
                                                         << myoMuscle.name << "' SEE element.");
                        return false;
                    }
                    // length
                    TiXmlElement *length_child = SEE_child->FirstChildElement("length");
                    if (length_child) {
                        if (sscanf(length_child->GetText(), "%lf", &myoMuscle.see.length) != 1) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading length constant");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No length element found in myoMuscle '"
                                                         << myoMuscle.name << "' SEE element.");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No SEE element found in myoMuscle '" << myoMuscle.name << "'.");
                    return false;
                }

            } else {
                ROS_ERROR_STREAM_NAMED("parser", "No name attribute specified for myoMuscle.");
                return false;
            }
            myoMuscles.push_back(myoMuscle);
        }
        return true;
    }
}

GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::RoboySim)
