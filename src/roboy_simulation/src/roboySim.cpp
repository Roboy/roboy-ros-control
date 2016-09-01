#include "roboySim.hpp"

namespace gazebo_ros_control {

    RoboySim::RoboySim() {

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "RoboySim",
                      ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
        }

        nh = new ros::NodeHandle;

        init_sub = nh->subscribe("/roboy/initialize", 1, &RoboySim::initializeControllers, this);
        record_sub = nh->subscribe("/roboy/record", 1, &RoboySim::record, this);
        recordResult_pub = nh->advertise<common_utilities::RecordResult>("/roboy/recordResult", 1000);
        steer_recording_sub = nh->subscribe("/roboy/steer_record", 1000, &RoboySim::steer_record, this);

        cmd = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
        pos = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
        vel = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
        eff = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
    }

    RoboySim::~RoboySim() {
        delete nh;
        delete cm;
        delete[] cmd, pos, vel, eff;
        update_thread->join();
        delete update_thread;
        delete walkController;
    }

    void RoboySim::initializeControllers(const common_utilities::Initialize::ConstPtr &msg) {
        initialized = false;
        sim_muscles.clear();

        // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
        // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
        const ros::NodeHandle joint_limit_nh(*nh);

        vector<string> start_controllers;
        for (uint i = 0; i < msg->controllers.size(); i++) {
            char name[20];
            sprintf(name, "motor%d", msg->controllers[i].id);

            // connect and register the joint state interface
            start_controllers.push_back(name);
            hardware_interface::JointStateHandle state_handle(name, &pos[msg->controllers[i].id],
                                                              &vel[msg->controllers[i].id],
                                                              &eff[msg->controllers[i].id]);
            jnt_state_interface.registerHandle(state_handle);

            try {
                ROS_INFO("Loading Dummy Muscle Plugin");
                sim_muscles.push_back(class_loader->createInstance("roboy_simulation::DummyMusclePlugin"));
                sim_muscles.back()->Init(myoMuscles[i]);
            }
            catch (pluginlib::PluginlibException &ex) {
                //handle the class failing to load
                ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
            }

            hardware_interface::JointHandle joint_handle(jnt_state_interface.getHandle(name),
                                                         &sim_muscles.back()->cmd);

            switch ((uint) msg->controllers[i].controlmode) {
                case 1: {
                    ROS_INFO("%s position controller", name);
                    // connect and register the joint position interface
                    jnt_pos_interface.registerHandle(joint_handle);
                    break;
                }
                case 2: {
                    ROS_INFO("%s velocity controller", name);
                    // connect and register the joint position interface
                    jnt_vel_interface.registerHandle(joint_handle);
                    break;
                }
                case 3: {
                    ROS_INFO("%s force controllers", name);
                    // connect and register the joint position interface
                    jnt_eff_interface.registerHandle(joint_handle);
                    break;
                }
                default:
                    ROS_WARN(
                            "The requested controlMode is not available, choose [1]PositionController [2]VelocityController [3]ForceController");
                    break;
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

        walkController = new WalkController(sim_muscles, parent_model);

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

            ROS_INFO_STREAM_NAMED("gazebo_ros_control", "Desired controller update period: " << control_period);
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

        // Create the controller manager
        ROS_INFO_STREAM_NAMED("ros_control_plugin", "Loading controller_manager");
        cm = new controller_manager::ControllerManager(this, *nh);

        // Initialize the emergency stop code.
        e_stop_active = false;
        last_e_stop_active = false;
        if (sdf_->HasElement("eStopTopic")) {
            const std::string e_stop_topic = sdf_->GetElement("eStopTopic")->Get<std::string>();
            e_stop_sub = nh->subscribe(e_stop_topic, 1, &RoboySim::eStopCB, this);
        }

        ROS_INFO_NAMED("gazebo_ros_control", "Starting gazebo_ros_control plugin in namespace: %s",
                       robot_namespace.c_str());

        ROS_INFO_NAMED("gazebo_ros_control", "Parsing myoMuscles");
        if (!parseMyoMuscleSDF(sdf_->ToString(""), myoMuscles))
            ROS_WARN_NAMED("gazebo_ros_control", "ERROR parsing myoMuscles, check your sdf file.");
        numberOfMyoMuscles = myoMuscles.size();
        ROS_INFO("Found %d MyoMuscles in sdf file", numberOfMyoMuscles);

        // class laoder for loading muscle plugins
        class_loader.reset(new pluginlib::ClassLoader<roboy_simulation::DummyMusclePlugin>
                                   ("roboy_simulation",
                                    "roboy_simulation::DummyMusclePlugin"));

        // Listen to the update event. This event is broadcast every simulation iteration.
        update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&RoboySim::Update, this));

        ROS_INFO_NAMED("gazebo_ros_control", "Loaded gazebo_ros_control.");
    }

    void RoboySim::readSim(ros::Time time, ros::Duration period) {
        ROS_DEBUG("read simulation");
        // update muscle plugins
        for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
            sim_muscles[muscle]->Update(time, period);
        }

        if(walkController->visualizeTendon)
            walkController->publishTendon();
        if(walkController->visualizeForce)
            walkController->publishForce();
        if(walkController->visualizeCOM)
            walkController->publishCOM();
        if(walkController->visualizeMomentArm)
            walkController->publishMomentArm();
    }

    void RoboySim::writeSim(ros::Time time, ros::Duration period) {
        ROS_DEBUG("write simulation");
        // apply the calculated forces
        for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
            uint j = 0;
            for (uint i = 0; i < sim_muscles[muscle]->force.size(); i++) {
                if(sim_muscles[muscle]->force[i].GetLength()>0.0){  // using zero forces makes the model collapse
                    sim_muscles[muscle]->links[j]->AddForceAtWorldPosition(-sim_muscles[muscle]->force[i],
                                                                           sim_muscles[muscle]->viaPointsInGlobalFrame[i]);
                    sim_muscles[muscle]->links[j]->AddForceAtWorldPosition(sim_muscles[muscle]->force[i],
                                                                           sim_muscles[muscle]->viaPointsInGlobalFrame[
                                                                                   i + 1]);
                    if (i == sim_muscles[muscle]->link_index - 1) {
                        j++;
                    }
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
            if(initialized)
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
        if(initialized)
            writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros);
        last_write_sim_time_ros = sim_time_ros;
    }

    void RoboySim::Reset() {
        // Reset timing variables to not pass negative update periods to controllers on world reset
        last_update_sim_time_ros = ros::Time();
        last_write_sim_time_ros = ros::Time();
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

    void RoboySim::eStopCB(const std_msgs::BoolConstPtr &e_stop_active) {

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
                uint link_index = 0;
                bool first_link = true;
                for (link_child_it = myoMuscle_it->FirstChildElement("link"); link_child_it;
                     link_child_it = link_child_it->NextSiblingElement("link")) {
                    string linkname = link_child_it->Attribute("name");
                    if (!linkname.empty()) {
                        TiXmlElement *viaPoint_child_it = NULL;
                        for (viaPoint_child_it = link_child_it->FirstChildElement("viaPoint"); viaPoint_child_it;
                             viaPoint_child_it = viaPoint_child_it->NextSiblingElement("viaPoint")) {
                            float x, y, z;
                            if (sscanf(viaPoint_child_it->GetText(), "%f %f %f", &x, &y, &z) != 3) {
                                ROS_ERROR_STREAM_NAMED("parser", "error reading [via point] (x y z)");
                                return false;
                            } else {
                                myoMuscle.viaPoints.push_back(math::Vector3(x, y, z));
                                link_index++;
                            }
                        }
                        myoMuscle.links.push_back(parent_model->GetLink(linkname));
                        if(first_link) {
                            myoMuscle.link_index = link_index;
                            first_link=false;
                        }
                        if (myoMuscle.viaPoints.empty()) {
                            ROS_ERROR_STREAM_NAMED("parser", "No viaPoint element found in myoMuscle '"
                                                             << myoMuscle.name << "' link element.");
                            return false;
                        }
                        if(myoMuscle.links.size()>2){
                            ROS_WARN_STREAM_NAMED("parser", "In myoMuscle '"
                                    << myoMuscle.name << "' link element: Only two links are supported.");
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No link name attribute specified for myoMuscle'"
                                                         << myoMuscle.name << "'.");
                        continue;
                    }
                }
                ROS_INFO("%ld viaPoints for myoMuscle %s", myoMuscle.viaPoints.size(), myoMuscle.name.c_str() );

                TiXmlElement *spans_joint_child_it = NULL;
                for (spans_joint_child_it = myoMuscle_it->FirstChildElement("spans_joint"); spans_joint_child_it;
                     spans_joint_child_it = spans_joint_child_it->NextSiblingElement("spans_joint")) {
                    string jointname = spans_joint_child_it->Attribute("name");
                    if (!jointname.empty()) {
                        myoMuscle.joint = parent_model->GetJoint(jointname);
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No spans_joint name attribute specified for myoMuscle'"
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
                    ROS_DEBUG_STREAM_NAMED("parser", "No motor element found in myoMuscle '" << myoMuscle.name <<
                                                    "', using default parameters");
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
                    ROS_DEBUG_STREAM_NAMED("parser", "No gear element found in myoMuscle '" << myoMuscle.name <<
                                                    "', using default parameters");
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
                    ROS_DEBUG_STREAM_NAMED("parser",
                                          "No spindle element found in myoMuscle '" << myoMuscle.name <<
                                          "', using default parameters");
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
                    ROS_DEBUG_STREAM_NAMED("parser", "No SEE element found in myoMuscle '" << myoMuscle.name <<
                                                    "', using default parameters");
                }

            } else {
                ROS_ERROR_STREAM_NAMED("parser",
                                       "No name attribute specified for myoMuscle, please name the muscle in sdf file");
                return false;
            }
            myoMuscles.push_back(myoMuscle);
        }
        return true;
    }

}

GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::RoboySim)
