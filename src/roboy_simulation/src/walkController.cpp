#include <std_msgs/Float32.h>
#include "walkController.hpp"

WalkController::WalkController() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "WalkController",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    force_torque_ankle_left_sub = nh->subscribe("/roboy/force_torque_ankle_left", 1,
                                                &WalkController::finite_state_machine, this);
    force_torque_ankle_right_sub = nh->subscribe("/roboy/force_torque_ankle_right", 1,
                                                 &WalkController::finite_state_machine, this);

    roboy_visualization_control_sub = nh->subscribe("/roboy/visualization_control", 10,
                                                    &WalkController::visualization_control, this);

    visualizeTendon_pub = nh->advertise<roboy_simulation::Tendon>("/visual/tendon", 1);
    marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);
    leg_state_pub = nh->advertise<roboy_simulation::LegState>("/roboy/leg_state", 2);
    roboyID_pub = nh->advertise<std_msgs::Int32>("/roboy/id",1);
    simulation_state_pub = nh->advertise<roboy_simulation::SimulationState>("/roboy/simulationState", 1);
    abort_pub = nh->advertise<roboy_simulation::Abortion>("/roboy/abort", 1000);
    toggle_walk_controller_sub = nh->subscribe("/roboy/toggle_walk_controller", 10,
                                               &WalkController::toggleWalkController, this);

    // the following links are part of my robot (this is useful if the model.sdf contains additional links)
    link_names.push_back("hip");
    link_names.push_back("thigh_left");
    link_names.push_back("thigh_right");
    link_names.push_back("shank_left");
    link_names.push_back("shank_right");
    link_names.push_back("foot_left");
    link_names.push_back("foot_right");

    leg_state[LEG::LEFT] = Stance;
    leg_state[LEG::RIGHT] = Swing;

    roboyID = gazebo::physics::getUniqueId();
}

WalkController::~WalkController() {
}

void WalkController::Load(gazebo::physics::ModelPtr parent_, sdf::ElementPtr sdf_) {
    ROS_INFO("Loading WalkController plugin");
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
        ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
    }

    // Get namespace for nodehandle
    if (sdf_->HasElement("robotNamespace")) {
        robot_namespace = sdf_->GetElement("robotNamespace")->Get<std::string>();
    } else {
        robot_namespace = parent_model->GetName(); // default
    }

    // Get robot_description ROS param name
    if (sdf_->HasElement("robotParam")) {
        robot_description = sdf_->GetElement("robotParam")->Get<std::string>();
    } else {
        robot_description = "robot_description"; // default
    }

    gazebo_max_step_size = parent_model->GetWorld()->GetPhysicsEngine()->GetMaxStepSize();

    // Get the Gazebo simulation period
    ros::Duration gazebo_period = ros::Duration(gazebo_max_step_size);

    // Decide the plugin control period
    control_period = ros::Duration(0.1);
    if (sdf_->HasElement("controlPeriod")) {
        control_period = ros::Duration(sdf_->Get<double>("controlPeriod"));

        ROS_INFO_STREAM("Desired controller update period: " << control_period);
        // Check the period against the simulation period
        if (control_period < gazebo_period) {
            ROS_ERROR_STREAM("Desired controller update period (" << control_period
                                                                  << " s) is faster than the gazebo simulation period ("
                                                                  <<
                                                                  gazebo_period << " s).");
        } else if (control_period > gazebo_period) {
            ROS_WARN_STREAM("Desired controller update period (" << control_period
                                                                 << " s) is slower than the gazebo simulation period ("
                                                                 <<
                                                                 gazebo_period << " s).");
        }
    } else {
        control_period = gazebo_period;
        ROS_DEBUG_STREAM("Control period not found in URDF/SDF, defaulting to Gazebo period of "
                                 << control_period);
    }

    // Create the controller manager
    ROS_INFO_STREAM_NAMED("ros_control_plugin", "Loading controller_manager");
//        cm = new controller_manager::ControllerManager(this, *nh);

    // Initialize the emergency stop code.
    e_stop_active = false;
    last_e_stop_active = false;
    if (sdf_->HasElement("eStopTopic")) {
        const std::string e_stop_topic = sdf_->GetElement("eStopTopic")->Get<std::string>();
        e_stop_sub = nh->subscribe(e_stop_topic, 1, &WalkController::eStopCB, this);
    }

    ROS_INFO("Parsing myoMuscles");
    if (!parseMyoMuscleSDF(sdf_->ToString(""), myoMuscles))
        ROS_WARN("ERROR parsing myoMuscles, check your sdf file.");
    numberOfMyoMuscles = myoMuscles.size();
    ROS_INFO("Found %d MyoMuscles in sdf file", numberOfMyoMuscles);

    // class laoder for loading muscle plugins
    class_loader.reset(new pluginlib::ClassLoader<roboy_simulation::DummyMusclePlugin>
                               ("roboy_simulation",
                                "roboy_simulation::DummyMusclePlugin"));

    sim_muscles.clear();
    for (uint muscle = 0; muscle < myoMuscles.size(); muscle++) {
        try {
            ROS_INFO("Loading Dummy Muscle Plugin for %s",
                           myoMuscles[muscle].name.c_str());
            sim_muscles.push_back(class_loader->createInstance("roboy_simulation::DummyMusclePlugin"));
            sim_muscles.back()->Init(myoMuscles[muscle], roboyID);

            muscles_spanning_joint[sim_muscles[muscle]->joint->GetName()].push_back(muscle);

            // initialize the queue for delayed activities depending on the spanning joint
            // hip muscles: 5ms, knee muscles 10ms, ankle muscles 20ms
            if(sim_muscles[muscle]->joint->GetName().find("groin")!=string::npos){
                for(uint i=0;i<5e-03/gazebo_max_step_size;i++){
                    activity[sim_muscles[muscle]->name].push_back(0.0);
                }
            }else if(sim_muscles[muscle]->joint->GetName().find("knee")!=string::npos) {
                for (uint i = 0; i < 10e-03 / gazebo_max_step_size; i++) {
                    activity[sim_muscles[muscle]->name].push_back(0.0);
                }
            }else if(sim_muscles[muscle]->joint->GetName().find("ankle")!=string::npos) {
                for (uint i = 0; i < 20e-03 / gazebo_max_step_size; i++) {
                    activity[sim_muscles[muscle]->name].push_back(0.0);
                }
            }

            a[sim_muscles[muscle]->name] = 0.0;
        }
        catch (pluginlib::PluginlibException &ex) {
            //handle the class failing to load
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
    }

    hip_CS = CoordSys(new CoordinateSystem(parent_model->GetLink("hip")));

    updateFootDisplacementAndVelocity();

    initializeValues();

    publishID();

    ros::spinOnce();

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&WalkController::Update, this));

    ROS_INFO("WalkController ready");
}

void WalkController::Update() {
    // Get the simulation time and period
    gz_time_now = parent_model->GetWorld()->GetSimTime();
    ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
    ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros;

    updateFootDisplacementAndVelocity();

    if(getLegInState(Stance)==LEG::NONE) {
        ROS_WARN_THROTTLE(1.0, "legs are not touching the ground");
    }

    if(control) {
        // Compute the controller commands
        updateTargetFeatures();
        updateMuscleForces();
        updateMuscleActivities();
    }else{ // relax the muscles
        for(uint muscle=0; muscle<sim_muscles.size(); muscle++){
            sim_muscles[muscle]->cmd = 0;
        }
    }

    // Check if we should update the controllers
    if (sim_period >= control_period) {
        // Store this simulation time
        last_update_sim_time_ros = sim_time_ros;

        // Update the robot simulation with the state of the gazebo model
        readSim(sim_time_ros, sim_period);
    }

    // Update the gazebo model with the result of the controller
    // computation
    writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros);
    last_write_sim_time_ros = sim_time_ros;

    // publish every now and then
    if((gz_time_now.nsec*gz_time_now.nsInMs)%100){
        message_counter = 1000;
        if (visualizeTendon)
            publishTendon();
        if (visualizeForce)
            publishForce();
        if (visualizeCOM)
            publishCOM();
        if (visualizeMomentArm)
            publishMomentArm();
        if(visualizeMesh)
            publishModel();
        if(visualizeStateMachineParameters)
            publishStateMachineParameters();
        if(visualizeCoordinateSystems)
            publishCoordinateSystems(parent_model->GetLink("hip"), ros::Time::now(), false);

        publishSimulationState();
        publishID();
        publishLegState();
    }

    checkAbort();

    ros::spinOnce();
}

void WalkController::readSim(ros::Time time, ros::Duration period) {
    ROS_DEBUG("read simulation");
    // update muscle plugins
    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        sim_muscles[muscle]->Update(time, period);
    }
}

void WalkController::writeSim(ros::Time time, ros::Duration period) {
    ROS_DEBUG("write simulation");
    // apply the calculated forces
    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        uint j = 0;
        for (uint i = 0; i < sim_muscles[muscle]->force.size(); i++) {
            if (sim_muscles[muscle]->force[i].GetLength() > 0.0) {  // using zero forces makes the model collapse
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

void WalkController::Reset() {
    // Reset timing variables to not pass negative update periods to controllers on world reset
    last_update_sim_time_ros = ros::Time();
    last_write_sim_time_ros = ros::Time();
}

void WalkController::eStopCB(const std_msgs::BoolConstPtr &msg) {
    e_stop_active = msg->data;
}

bool WalkController::parseMyoMuscleSDF(const string &sdf, vector<roboy_simulation::MyoMuscleInfo> &myoMuscles) {
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
                    if (first_link) {
                        myoMuscle.link_index = link_index;
                        first_link = false;
                    }
                    if (myoMuscle.viaPoints.empty()) {
                        ROS_ERROR_STREAM_NAMED("parser", "No viaPoint element found in myoMuscle '"
                                << myoMuscle.name << "' link element.");
                        return false;
                    }
                    if (myoMuscle.links.size() > 2) {
                        ROS_WARN_STREAM_NAMED("parser", "In myoMuscle '"
                                << myoMuscle.name << "' link element: Only two links are supported.");
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No link name attribute specified for myoMuscle'"
                            << myoMuscle.name << "'.");
                    continue;
                }
            }
            ROS_INFO("%ld viaPoints for myoMuscle %s", myoMuscle.viaPoints.size(), myoMuscle.name.c_str());

            TiXmlElement *spans_joint_child_it = NULL;
            for (spans_joint_child_it = myoMuscle_it->FirstChildElement("spans_joint"); spans_joint_child_it;
                 spans_joint_child_it = spans_joint_child_it->NextSiblingElement("spans_joint")) {
                string jointname = spans_joint_child_it->Attribute("name");
                if (!jointname.empty()) {
                    myoMuscle.spanning_joint = parent_model->GetJoint(jointname);
                    if(strcmp(spans_joint_child_it->GetText(),"extensor")==0)
                        myoMuscle.muscle_type = EXTENSOR;
                    else if(strcmp(spans_joint_child_it->GetText(),"flexor")==0)
                        myoMuscle.muscle_type = FLEXOR;
                    else if(strcmp(spans_joint_child_it->GetText(),"stabilizer")==0)
                        myoMuscle.muscle_type = STABILIZER;
                    else
                        ROS_WARN_STREAM_NAMED("parser", "muscle type not defined for " << myoMuscle.name << "'.");
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
    TiXmlElement *foot_sole_left = NULL;
    foot_sole_left = root->FirstChildElement("foot_sole_left");
    if (foot_sole_left) {
        float x, y, z;
        if (sscanf(foot_sole_left->GetText(), "%lf %lf %lf", &foot_sole[LEG::LEFT].x, &foot_sole[LEG::LEFT].y,
                   &foot_sole[LEG::LEFT].z) != 3) {
            ROS_ERROR_STREAM_NAMED("parser", "error reading [foot_sole_left] (x y z)");
            return false;
        }
    } else {
        ROS_WARN_STREAM_NAMED("parser", "No foot_sole_left element found, using " << foot_sole[LEG::LEFT]);
    }
    TiXmlElement *foot_sole_right = NULL;
    foot_sole_right = root->FirstChildElement("foot_sole_right");
    if (foot_sole_right) {
        float x, y, z;
        if (sscanf(foot_sole_right->GetText(), "%lf %lf %lf", &foot_sole[LEG::RIGHT].x, &foot_sole[LEG::RIGHT].y,
                   &foot_sole[LEG::RIGHT].z) != 3) {
            ROS_ERROR_STREAM_NAMED("parser", "error reading [foot_sole_right] (x y z)");
            return false;
        }
    } else {
        ROS_WARN_STREAM_NAMED("parser", "No foot_sole_right element found, using " << foot_sole[LEG::RIGHT]);
    }

    ROS_INFO_STREAM("foot_soles: " << foot_sole[LEG::LEFT] << " / " << foot_sole[LEG::RIGHT]);

    return true;
}

void WalkController::calculateCOM(int type, math::Vector3 &COM) {
    double mass_total = 0;
    COM = math::Vector3(0, 0, 0);
    for (auto link_name:link_names) {
        physics::LinkPtr link = parent_model->GetLink(link_name);
        double m = link->GetInertial()->GetMass();
        mass_total += m;
        switch (type) {
            case POSITION: {
                math::Pose p = link->GetWorldCoGPose();
                COM += p.pos * m;
                break;
            }
            case VELOCITY: {
                math::Vector3 v = link->GetWorldCoGLinearVel();
                COM += v * m;
                break;
            }
        }
    }
    COM /= mass_total;
}

vector<double> WalkController::calculateAngles(vector<pair<std::string, std::string>> linkpair, PLANE flag) {
    vector<double> angle;
    for (auto compare : linkpair) {
        physics::LinkPtr link1;
        link1 = parent_model->GetLink(compare.first);
        physics::LinkPtr link2;
        link2 = parent_model->GetLink(compare.second);
        math::Pose p1 = link1->GetWorldCoGPose();
        math::Pose p2 = link2->GetWorldCoGPose();
        math::Vector3 Euler1 = p1.rot.GetAsEuler();
        math::Vector3 Euler2 = p2.rot.GetAsEuler();
        switch (flag) {
            case SAGITTAL:
                angle.push_back(Euler1.y - Euler2.y);
                break;
            case CORONAL:
                angle.push_back(Euler1.x - Euler2.x);
                break;
            case TRAVERSAL:
                angle.push_back(Euler1.z - Euler2.z);
                break;
        }
    }
    return angle;
}

double WalkController::calculateAngle(string link0, string link1, PLANE flag) {
    math::Pose p1 = parent_model->GetLink(link0)->GetWorldCoGPose();
    math::Pose p2 = parent_model->GetLink(link1)->GetWorldCoGPose();
    math::Vector3 Euler1 = p1.rot.GetAsEuler();
    math::Vector3 Euler2 = p2.rot.GetAsEuler();
    switch (flag) {
        case SAGITTAL:
            return Euler1.y - Euler2.y;
        case CORONAL:
            return Euler1.x - Euler2.x;
        case TRAVERSAL:
            return Euler1.z - Euler2.z;
        default:
            ROS_ERROR("unknown plane");
    }
}

map<string, math::Vector3> WalkController::calculateTrunk() {
    physics::LinkPtr trunk = parent_model->GetLink("hip");
    math::Pose p = trunk->GetWorldCoGPose();
    math::Vector3 Euler = p.rot.GetAsEuler();
    math::Vector3 velocity = trunk->GetWorldCoGLinearVel();
    map<string, math::Vector3> r;
    string a("Velocity");
    string b("Angle");
    r.insert({a, velocity});
    r.insert({b, Euler});
    return r;
}

void WalkController::initializeValues(){
    math::Vector3 euler = parent_model->GetLink("hip")->GetWorldPose().rot.GetAsEuler();
    phi_trunk_0 = euler.x;
    theta_trunk_0 = euler.y;

    calculateCOM(POSITION, initial_center_of_mass_height);

    euler = parent_model->GetLink("thigh_left")->GetWorldPose().rot.GetAsEuler();
    phi_groin_0[LEG::LEFT] = euler.x;
    theta_groin_0[LEG::LEFT] = euler.y;

    euler = parent_model->GetLink("thigh_right")->GetWorldPose().rot.GetAsEuler();
    phi_groin_0[LEG::RIGHT] = euler.x;
    theta_groin_0[LEG::RIGHT] = euler.y;

    k_v = 0.1;
    k_h = 0.1;
    k_p_theta_left[0] = 0.1;
    k_p_theta_left[1] = 0.1;
    k_p_theta_left[2] = 0.1;
    k_p_theta_left[3] = 0.1;
    k_p_theta_right[0] = 0.1;
    k_p_theta_right[1] = 0.1;
    k_p_theta_right[2] = 0.1;
    k_p_theta_right[3] = 0.1;
    k_d_theta_left[0] = 0.1;
    k_d_theta_left[1] = 0.1;
    k_d_theta_left[2] = 0.1;
    k_d_theta_left[3] = 0.1;
    k_d_theta_right[0] = 0.1;
    k_d_theta_right[1] = 0.1;
    k_d_theta_right[2] = 0.1;
    k_d_theta_right[3] = 0.1;
    k_p_phi[0] = 0.1;
    k_p_phi[1] = 0.1;
    k_d_phi[0] = 0.1;
    k_d_phi[1] = 0.1;
    // target force torque gains
    k_V = 0.1;
    k_P = 0.1;
    k_Q = 0.1;
    k_omega = 0.1;
}

void WalkController::updateFootDisplacementAndVelocity(){
    // calculate the COM position and velocity
    calculateCOM(POSITION, center_of_mass[POSITION]);
    calculateCOM(VELOCITY, center_of_mass[VELOCITY]);

    // calculate foot_sole position in world frame
    math::Pose foot_pose = parent_model->GetLink(FOOT[LEG::LEFT])->GetWorldPose();
    foot_sole_global[LEG::LEFT] = foot_pose.pos + foot_pose.rot.RotateVector(foot_sole[LEG::LEFT]);
    foot_pose = parent_model->GetLink(FOOT[LEG::RIGHT])->GetWorldPose();
    foot_sole_global[LEG::RIGHT] = foot_pose.pos + foot_pose.rot.RotateVector(foot_sole[LEG::RIGHT]);

    // update coordinate system wrt heading
    hip_CS->UpdateHeading();

    // v_COM is the velocity of center_of_mass projected onto forward direction
    v_COM = center_of_mass[VELOCITY].Dot(hip_CS->Xn);

    // calculate signed horizontal distance between foot_pos and COM
    for(uint leg=LEG::LEFT; leg<=LEG::RIGHT; leg++) {
        // get the foot velocity
        math::Vector3 foot_vel = parent_model->GetLink(FOOT[leg])->GetWorldCoGLinearVel();

        // calculate the distance and velocity between foot and center of mass in world frame
        d_foot_pos[leg] = foot_sole_global[leg] - center_of_mass[POSITION];
        d_foot_vel[leg] = foot_vel - center_of_mass[VELOCITY];

        // rotate into hip coordinate system
        d_foot_pos[leg] = hip_CS->rot.RotateVector(d_foot_pos[leg]);
        d_foot_vel[leg] = hip_CS->rot.RotateVector(d_foot_vel[leg]);

        // calculate signed sagittal and coronal foot displacement and velocity
        d_s[leg] = d_foot_pos[leg].Dot(hip_CS->X);
        d_c[leg] = d_foot_pos[leg].Dot(hip_CS->Y);
        v_s[leg] = d_foot_vel[leg].Dot(hip_CS->X);
        v_c[leg] = d_foot_vel[leg].Dot(hip_CS->Y);
    }
}

void WalkController::finite_state_machine(const roboy_simulation::ForceTorque::ConstPtr &msg) {
    // check what state the leg is currently in
    bool state_transition = false;

    switch (leg_state[msg->leg]) {
        case Stance: {
            if (d_s[msg->leg] < d_lift || (leg_state[LEG::LEFT] == Stance && leg_state[LEG::RIGHT] == Stance)) {
                state_transition = true;
                ROS_INFO("ds: %f d_lift: %f", d_s[msg->leg] , d_lift);
            }
            break;
        }
        case Lift_off: {
            double force_norm = sqrt(pow(msg->force.x, 2.0) + pow(msg->force.y, 2.0) + pow(msg->force.z, 2.0));
            if (force_norm < F_contact) {
                state_transition = true;
                ROS_INFO("force_norm: %f F_contact: %f", force_norm, F_contact);
            }
            break;
        }
        case Swing: {
            if (d_s[msg->leg] > d_prep) {
                state_transition = true;
                ROS_INFO("d_s: %f d_prep: %f", d_s[msg->leg] , d_prep);
            }
            break;
        }
        case Stance_Preparation: {
            double force_norm = sqrt(pow(msg->force.x, 2.0) + pow(msg->force.y, 2.0) + pow(msg->force.z, 2.0));
            if (force_norm > F_contact) {
                state_transition = true;
                ROS_INFO("force_norm: %f F_contact: %f", force_norm, F_contact);
            }
            break;
        }
    }

    if (state_transition) {
        LEG_STATE new_state = NextState(leg_state[msg->leg]);
        ROS_INFO("%s state transition \t%s \t-> \t%s", LEG_NAMES_STRING[msg->leg],
                 LEG_STATE_STRING[leg_state[msg->leg]],
                 LEG_STATE_STRING[new_state] );
        leg_state[msg->leg] = new_state;
    }

    if(visualizeForceTorqueSensors){
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "world";
        char forcetorquenamespace[20];
        sprintf(forcetorquenamespace, "forceTorqueSensors_%d", roboyID);
        arrow.ns = forcetorquenamespace;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.color.r = 1.0f;
        arrow.color.g = 1.0f;
        arrow.color.b = 0.0f;
        arrow.lifetime = ros::Duration();
        arrow.scale.x = 0.005;
        arrow.scale.y = 0.03;
        arrow.scale.z = 0.03;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.color.a = 1.0;
        arrow.id = 10000 + msg->leg;
        arrow.header.stamp = ros::Time::now();
        arrow.points.clear();
        geometry_msgs::Point p;
        math::Vector3 pos = parent_model->GetJoint(msg->joint)->GetWorldPose().pos;
        p.x = pos.x;
        p.y = pos.y;
        p.z = pos.z;
        arrow.points.push_back(p);
        p.x += msg->force.x;
        p.y += msg->force.y;
        p.z += msg->force.z;
        arrow.points.push_back(p);
        marker_visualization_pub.publish(arrow);
//        text.id = message_counter++;
//        text.pose.position = p;
//        text.text = "foot_sole[left]";
//        marker_visualization_pub.publish(text);
//
//        visualization_msgs::Marker text;
//        text.header.frame_id = "world";
//        text.ns = momentarmnamespace;
//        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//        text.color.r = 1.0;
//        text.color.g = 1.0;
//        text.color.b = 1.0;
//        text.color.a = 1.0;
//        text.lifetime = ros::Duration();
//        text.scale.z = 0.03;
//        text.action = visualization_msgs::Marker::ADD;
//        text.id = message_counter++;
//        text.header.stamp = ros::Time::now();
//        text.pose.orientation.w = 1.0;
    }
}

LEG_STATE WalkController::NextState(LEG_STATE s) {
    LEG_STATE newstate;
    switch (s) {
        case Stance:
            newstate = Lift_off;
            break;
        case Lift_off:
            newstate = Swing;
            break;
        case Swing:
            newstate = Stance_Preparation;
            break;
        case Stance_Preparation:
            newstate = Stance;
            break;
    }
    return newstate;
}

LEG WalkController::getLegInState(LEG_STATE s) {
    if (leg_state[LEG::LEFT] == s)
        return LEG::LEFT;
    else if (leg_state[LEG::RIGHT] == s)
        return LEG::RIGHT;
    else
        return LEG::NONE;
}

void WalkController::updateTargetFeatures() {
    // target velocity
    math::Vector3 v_target(v_forward, 0, 0);
    v_target = hip_CS->rot.RotateVector(v_target);

    math::Pose hip_pose = parent_model->GetLink("hip")->GetWorldPose();

    // trunk
    double theta_trunk = theta_trunk_0 + k_v * (v_forward - v_COM);
    double phi_trunk = phi_trunk_0 + k_h * (psi_heading - hip_pose.rot.GetYaw());
    // target orientation for the hip
    Q["hip"] = math::Quaternion(phi_trunk, theta_trunk, psi_heading);
    // there is no target position for the hip, so we set it to the current position
    P["hip"] = hip_pose.pos;
    // target velocity is the forward velocity into desired heading direction
    v["hip"] = v_target;
    // target angular velocity is zero
    omega["hip"] = math::Vector3::Zero;

    // thigh left
    physics::LinkPtr thigh_left = parent_model->GetLink("thigh_left");
    double theta_groin_left = theta_groin_0[LEG::LEFT] + k_p_theta_left[leg_state[LEG::LEFT]] * d_s[LEG::LEFT]
                              + k_d_theta_left[leg_state[LEG::LEFT]] * (v_forward - v_s[LEG::LEFT]);
    double phi_groin_left = phi_groin_0[LEG::LEFT] + k_p_phi[LEG::LEFT] * d_c[LEG::LEFT] - k_d_phi[LEG::LEFT] * v_c[LEG::LEFT];
    // target orientation for the hip
    Q["thigh_left"] = math::Quaternion(phi_groin_left, theta_groin_left, psi_heading);
    // there is no target position, so we set it to the current position
    P["thigh_left"] = thigh_left->GetWorldPose().pos;
    // there is no target velocity, so we set it to the current velocity
    v["thigh_left"] = thigh_left->GetWorldCoGLinearVel();
    // target angular velocity is zero while stance preparation
    if (leg_state[LEG::LEFT] == Stance_Preparation)
        omega["thigh_left"] = math::Vector3::Zero;
    else  // there is no target
        omega["thigh_left"] = thigh_left->GetWorldAngularVel();

    // thigh right
    physics::LinkPtr thigh_right = parent_model->GetLink("thigh_right");
    double theta_groin_right = theta_groin_0[LEG::RIGHT] + k_p_theta_right[leg_state[LEG::RIGHT]] * d_s[LEG::LEFT]
                               + k_d_theta_right[leg_state[LEG::RIGHT]] * (v_forward - v_s[LEG::LEFT]);
    double phi_groin_right = phi_groin_0[LEG::RIGHT] + k_p_phi[LEG::RIGHT] * d_c[LEG::LEFT] - k_d_phi[LEG::RIGHT] * v_c[LEG::LEFT];
    // target orientation for the hip
    Q["thigh_right"] = math::Quaternion(phi_groin_right, theta_groin_right, psi_heading);
    // there is no target position, so we set it to the current position
    P["thigh_right"] = thigh_right->GetWorldPose().pos;
    // there is no target velocity, so we set it to the current velocity
    v["thigh_right"] = thigh_right->GetWorldCoGLinearVel();
    // target angular velocity is zero while stance preparation
    if (leg_state[LEG::RIGHT] == Stance_Preparation) // target is zero while stance preparation
        omega["thigh_right"] = math::Vector3::Zero;
    else  // there is no target
        omega["thigh_right"] = thigh_right->GetWorldAngularVel();

    // shank left
    physics::LinkPtr shank_left = parent_model->GetLink("shank_left");
    math::Pose shank_left_pose = shank_left->GetWorldPose();
    // target orientation for the hip
    Q["shank_left"] = math::Quaternion(shank_left_pose.rot.GetRoll(), theta_knee[LEG::LEFT], psi_heading);
    // there is no target position for the hip, so we set it to the current position
    P["shank_left"] = shank_left_pose.pos;
    // target velocity is the forward velocity into desired heading direction
    v["shank_left"] = shank_left->GetWorldCoGLinearVel();
    // target angular velocity is zero
    omega["shank_left"] = shank_left->GetWorldAngularVel();

    // shank right
    physics::LinkPtr shank_right = parent_model->GetLink("shank_right");
    math::Pose shank_right_pose = shank_right->GetWorldPose();
    // target orientation for the hip
    Q["shank_right"] = math::Quaternion(shank_right_pose.rot.GetRoll(), theta_knee[LEG::RIGHT], psi_heading);
    // there is no target position for the hip, so we set it to the current position
    P["shank_right"] = shank_right_pose.pos;
    // target velocity is the forward velocity into desired heading direction
    v["shank_right"] = shank_right->GetWorldCoGLinearVel();
    // target angular velocity is zero
    omega["shank_right"] = shank_right->GetWorldAngularVel();

    // foot left
    physics::LinkPtr foot_left = parent_model->GetLink("foot_left");
    math::Pose foot_left_pose = foot_left->GetWorldPose();
    // target orientation for the hip
    Q["foot_left"] = math::Quaternion(foot_left_pose.rot.GetRoll(), theta_ankle[LEG::LEFT], psi_heading);
    // there is no target position for the hip, so we set it to the current position
    P["foot_left"] = foot_left_pose.pos;
    // target velocity is the forward velocity into desired heading direction
    v["foot_left"] = foot_left->GetWorldCoGLinearVel();
    // target angular velocity is zero
    omega["foot_left"] = foot_left->GetWorldAngularVel();

    // foot right
    physics::LinkPtr foot_right = parent_model->GetLink("foot_right");
    math::Pose foot_right_pose = foot_left->GetWorldPose();
    // target orientation for the hip
    Q["foot_right"] = math::Quaternion(foot_right_pose.rot.GetRoll(), theta_ankle[LEG::RIGHT], psi_heading);
    // there is no target position for the hip, so we set it to the current position
    P["foot_right"] = foot_right_pose.pos;
    // target velocity is the forward velocity into desired heading direction
    v["foot_right"] = foot_right->GetWorldCoGLinearVel();
    // target angular velocity is zero
    omega["foot_right"] = foot_right->GetWorldAngularVel();
}

void WalkController::updateMuscleForces() {
    // calculate target force and torque
    for (auto link_name:link_names) {
        physics::LinkPtr link = parent_model->GetLink(link_name);
        F[link_name] = k_P * (P[link_name] - link->GetWorldPose().pos) +
                       k_V * (v[link_name] - link->GetWorldCoGLinearVel());
        math::Quaternion q = Q[link_name] * link->GetWorldPose().rot.GetInverse();
        math::Vector3 v(q.x, q.y, q.z);
        double norm = v.GetLength();
        math::Vector3 exponent = exp(q.w) * (cos(norm) * math::Vector3::One + v.Normalize() * sin(norm));
        T[link_name] = k_Q * (Q[link_name].GetAsMatrix3() * exponent) +
                       k_omega * (omega[link_name] - link->GetWorldAngularVel());

        // iterate over all joints of link
        physics::Joint_V joints = link->GetChildJoints();
        for (auto joint:joints) {
            // calculate the target Force of each muscle spanning this joint
            for (uint muscle:muscles_spanning_joint[joint->GetName()]) {
                math::Vector3 momentArm = sim_muscles[muscle]->momentArm;
                double length = momentArm.GetLength();
                math::Vector3 momentArm_normalized = momentArm.Normalize();
                tau[sim_muscles[muscle]->name] =
                        momentArm_normalized.Cross(link->GetWorldPose().pos - joint->GetWorldPose().pos)
                                .Dot(F[link_name]) + momentArm_normalized.Dot(T[link_name]);
                if(length>0.0)
                    F_tilde[sim_muscles[muscle]->name] = tau[sim_muscles[muscle]->name] / momentArm.GetLength();
                else
                    F_tilde[sim_muscles[muscle]->name] = 0.0;
            }
        }
    }
    ROS_INFO_STREAM("\nF_tilde: " << F_tilde["motor0"] << "\ntau: " << tau["motor0"] << "\nF: " <<
             F["motor0"] << "\nT: " <<  T["motor0"]);
}

void WalkController::updateMuscleActivities(){
    // queue the activities for time delayed feedbacks
    for(uint muscle=0; muscle<sim_muscles.size(); muscle++) {
        activity[sim_muscles[muscle]->name].push_back(F_tilde[sim_muscles[muscle]->name] / F_max);
    }
    // update the feedbacks. this depends on the current state of each leg
    for(uint leg=0; leg<=2; leg++){
        switch(leg_state[leg]){
            case Stance: {
                // during stance the hip muscles stabilize and rotate the hip towards its target orientation
                char joint[20];
                sprintf(joint, "groin_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    feedback[sim_muscles[muscle]->name] =
                            activity[sim_muscles[muscle]->name].front() + c_stance_lift;
                }
                // the rest of the leg has no target position or orientations, instead we are using positive
                // force feedback for the extensors to achieve joint compliance
                sprintf(joint, "knee_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                k_M_Fplus * activity[sim_muscles[muscle]->name].front() + c_stance_lift;
                }
                sprintf(joint, "ankle_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                k_M_Fplus * activity[sim_muscles[muscle]->name].front() + c_stance_lift;
                }
                break;
            }
            case Lift_off: {
                // during lift-off all muscles attached to the hip are fed a constant
                // excitation of high magnitude, to initiate a leg swing
                char joint[20];
                sprintf(joint, "groin_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] = c_hip_lift + c_stance_lift;
                    if (sim_muscles[muscle]->muscle_type == FLEXOR)
                        feedback[sim_muscles[muscle]->name] = -c_hip_lift + c_stance_lift;
                }
                // the rest of the leg has no target position or orientations, instead we are using positive
                // force feedback for the extensors to achieve joint compliance
                sprintf(joint, "knee_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                k_M_Fplus * activity[sim_muscles[muscle]->name].front() + c_stance_lift;
                }
                sprintf(joint, "ankle_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                k_M_Fplus * activity[sim_muscles[muscle]->name].front() + c_stance_lift;
                }
                break;
            }
            case Swing: {
                char joint[20];
                sprintf(joint, "groin_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    feedback[sim_muscles[muscle]->name] =
                            activity[sim_muscles[muscle]->name].front() + c_swing_prep;
                }
                sprintf(joint, "ankle_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                k_M_Fplus * activity[sim_muscles[muscle]->name].front() + c_swing_prep;
                }
                break;
            }
            case Stance_Preparation: {
                char joint[20];
                sprintf(joint, "groin_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    feedback[sim_muscles[muscle]->name] =
                            activity[sim_muscles[muscle]->name].front() + c_swing_prep;
                }
                sprintf(joint, "knee_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                k_M_Fplus * activity[sim_muscles[muscle]->name].front() + c_swing_prep;
                }
                sprintf(joint, "ankle_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                k_M_Fplus * activity[sim_muscles[muscle]->name].front() + c_swing_prep;
                }
                break;
            }
        }
    }
    // integrate the activity and feedback and pop the first activity
    // set command of each muscle to the integrated activity
    for(uint muscle=0; muscle<sim_muscles.size(); muscle++) {
        a[sim_muscles[muscle]->name] += 100.0*gazebo_max_step_size*
                        (feedback[sim_muscles[muscle]->name] - activity[sim_muscles[muscle]->name].front());
        activity[sim_muscles[muscle]->name].pop_front();
        sim_muscles[muscle]->cmd = a[sim_muscles[muscle]->name];
    }
    ROS_INFO("a: %lf, activity: %lf, feedback %lf", a["motor0"], activity["motor0"].front(),
             feedback["motor0"]);
}

void WalkController::updateEnergies(){

}

bool WalkController::checkAbort(){
    if(center_of_mass[POSITION].z<0.1*initial_center_of_mass_height.z) {
        roboy_simulation::Abortion msg;
        msg.roboyID = roboyID;
        msg.reason = COMheight;
        abort_pub.publish(msg);
        ROS_WARN_THROTTLE(1.0,"center of mass below threshold, aborting");
        return true;
    }
    if(radiansToDegrees(fabs(hip_CS->rot.GetYaw())-fabs(psi_heading)) > 45.0f) {
        roboy_simulation::Abortion msg;
        msg.roboyID = roboyID;
        msg.reason = headingDeviation;
        abort_pub.publish(msg);
        ROS_WARN_THROTTLE(1.0,"deviation from target heading above threshold, aborting");
        return true;
    }
    for(auto link_name:link_names){
        physics::Collision_V collisions = parent_model->GetLink(link_name)->GetCollisions();
        for(auto collision:collisions){
            physics::LinkPtr link = collision->GetLink();
            if(link->GetName().find("foot")!=string::npos || link->GetName().find("hip")!=string::npos)// collsions for
                // the feed are ok or hip?!
                continue;
            else {
                roboy_simulation::Abortion msg;
                msg.roboyID = roboyID;
                msg.reason = selfCollision;
                abort_pub.publish(msg);
                ROS_WARN_THROTTLE(1.0, "self collision detected with %s, aborting", link->GetName().c_str());
                return true;
            }
        }
    }
    return false;
}

void WalkController::visualization_control(const roboy_simulation::VisualizationControl::ConstPtr &msg) {
    if(msg->roboyID == roboyID) { // only react to messages with my roboyID
        switch (msg->control) {
            case Tendon: {
                visualizeTendon = msg->value;
                break;
            }
            case COM: {
                visualizeCOM = msg->value;
                break;
            }
            case Forces: {
                visualizeForce = msg->value;
                break;
            }
            case MomentArm: {
                visualizeMomentArm = msg->value;
                break;
            }
            case Mesh: {
                visualizeMesh = msg->value;
                break;
            }
            case StateMachineParameters: {
                visualizeStateMachineParameters = msg->value;

                break;
            }
            case CoordinateSystems: {
                visualizeCoordinateSystems = msg->value;
                break;
            }
            case ForceTorqueSensors: {
                visualizeForceTorqueSensors = msg->value;
                break;
            }
        }
        if(!visualizeTendon || !visualizeCOM || !visualizeForce || !visualizeMomentArm ||
                !visualizeMesh || !visualizeStateMachineParameters || !visualizeForceTorqueSensors){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.id = message_counter++;
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker_visualization_pub.publish(marker);
        }

    }
}

void WalkController::publishTendon() {
    static bool add = true;
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    char tendonnamespace[20];
    sprintf(tendonnamespace, "tendon_%d", roboyID);
    line_strip.ns = tendonnamespace;
    if (add) {
        line_strip.action = visualization_msgs::Marker::ADD;
        add = false;
    } else {
        line_strip.action = visualization_msgs::Marker::MODIFY;
    }
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.003;
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;
    line_strip.pose.orientation.w = 1.0;

    roboy_simulation::Tendon msg;
    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        line_strip.points.clear();
        line_strip.id = message_counter++;
        for (uint i = 0; i < sim_muscles[muscle]->viaPointsInGlobalFrame.size(); i++) {
            geometry_msgs::Vector3 vp;
            vp.x = sim_muscles[muscle]->viaPointsInGlobalFrame[i].x;
            vp.y = sim_muscles[muscle]->viaPointsInGlobalFrame[i].y;
            vp.z = sim_muscles[muscle]->viaPointsInGlobalFrame[i].z;
            msg.viaPoints.push_back(vp);
            geometry_msgs::Point p;
            p.x = sim_muscles[muscle]->viaPointsInGlobalFrame[i].x;
            p.y = sim_muscles[muscle]->viaPointsInGlobalFrame[i].y;
            p.z = sim_muscles[muscle]->viaPointsInGlobalFrame[i].z;
            line_strip.points.push_back(p);
        }
        marker_visualization_pub.publish(line_strip);
    }
    visualizeTendon_pub.publish(msg);
}

void WalkController::publishCOM() {
    static bool add = true;
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    char comnamespace[20];
    sprintf(comnamespace, "COM_%d", roboyID);
    sphere.ns = comnamespace;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.color.r = 0.0f;
    sphere.color.g = 0.0f;
    sphere.color.b = 1.0f;
    sphere.color.a = 1.0;
    sphere.lifetime = ros::Duration();
    sphere.scale.x = 0.1;
    sphere.scale.y = 0.1;
    sphere.scale.z = 0.1;
    if (add) {
        sphere.action = visualization_msgs::Marker::ADD;
        add = false;
    } else {
        sphere.action = visualization_msgs::Marker::MODIFY;
    }
    sphere.header.stamp = ros::Time::now();
    sphere.points.clear();
    sphere.id = message_counter++;
    sphere.pose.position.x = center_of_mass[POSITION].x;
    sphere.pose.position.y = center_of_mass[POSITION].y;
    sphere.pose.position.z = center_of_mass[POSITION].z;
    marker_visualization_pub.publish(sphere);
}

void WalkController::publishForce() {
    uint id = 99999999;
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    char forcenamespace[20];
    sprintf(forcenamespace, "force_%d", roboyID);
    arrow.ns = forcenamespace;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration();
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.03;
    arrow.scale.z = 0.03;
    arrow.action = visualization_msgs::Marker::ADD;

    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        for (uint i = 0; i < sim_muscles[muscle]->force.size(); i++) {
            // actio
            arrow.id = id++;
            arrow.color.r = 0.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.header.stamp = ros::Time::now();
            arrow.points.clear();
            geometry_msgs::Point p;
            p.x = sim_muscles[muscle]->viaPointsInGlobalFrame[i].x;
            p.y = sim_muscles[muscle]->viaPointsInGlobalFrame[i].y;
            p.z = sim_muscles[muscle]->viaPointsInGlobalFrame[i].z;
            arrow.points.push_back(p);
            p.x -= sim_muscles[muscle]->force[i].x;
            p.y -= sim_muscles[muscle]->force[i].y;
            p.z -= sim_muscles[muscle]->force[i].z;
            arrow.points.push_back(p);
            marker_visualization_pub.publish(arrow);
            // reactio
            arrow.id = id++;
            arrow.color.r = 1.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.header.stamp = ros::Time::now();
            arrow.points.clear();
            p.x = sim_muscles[muscle]->viaPointsInGlobalFrame[i + 1].x;
            p.y = sim_muscles[muscle]->viaPointsInGlobalFrame[i + 1].y;
            p.z = sim_muscles[muscle]->viaPointsInGlobalFrame[i + 1].z;
            arrow.points.push_back(p);
            p.x += sim_muscles[muscle]->force[i].x;
            p.y += sim_muscles[muscle]->force[i].y;
            p.z += sim_muscles[muscle]->force[i].z;
            arrow.points.push_back(p);
            marker_visualization_pub.publish(arrow);
        }
    }
}

void WalkController::publishMomentArm() {
    uint id = 999999;
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    char momentarmnamespace[20];
    sprintf(momentarmnamespace, "momentarm_%d", roboyID);
    arrow.ns = momentarmnamespace;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.r = 0.0f;
    arrow.color.g = 0.0f;
    arrow.color.b = 1.0f;
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration();
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.03;
    arrow.scale.z = 0.03;
    arrow.action = visualization_msgs::Marker::ADD;
    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        arrow.id = id++;
        arrow.header.stamp = ros::Time::now();
        arrow.points.clear();
        geometry_msgs::Point p;
        math::Pose jointPose = sim_muscles[muscle]->joint->GetWorldPose();
        p.x = jointPose.pos.x;
        p.y = jointPose.pos.y;
        p.z = jointPose.pos.z;
        arrow.points.push_back(p);
        p.x += sim_muscles[muscle]->momentArm.x;
        p.y += sim_muscles[muscle]->momentArm.y;
        p.z += sim_muscles[muscle]->momentArm.z;
        arrow.points.push_back(p);
        marker_visualization_pub.publish(arrow);
    }
}

void WalkController::publishModel(){
    visualization_msgs::Marker mesh;
    mesh.header.frame_id = "world";
    char modelnamespace[20];
    sprintf(modelnamespace, "model_%d", roboyID);
    mesh.ns = modelnamespace;
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.color.r = 1.0f;
    mesh.color.g = 1.0f;
    mesh.color.b = 1.0f;
    mesh.color.a = 0.5;
    mesh.scale.x = 1.0;
    mesh.scale.y = 1.0;
    mesh.scale.z = 1.0;
    mesh.lifetime = ros::Duration();
    mesh.header.stamp = ros::Time::now();
    for(auto link_name:link_names){
        mesh.id = message_counter++;
        physics::LinkPtr link = parent_model->GetLink(link_name);
        math::Pose pose = link->GetWorldPose();
        mesh.pose.position.x = pose.pos.x;
        mesh.pose.position.y = pose.pos.y;
        mesh.pose.position.z = pose.pos.z;
        mesh.pose.orientation.x = pose.rot.x;
        mesh.pose.orientation.y = pose.rot.y;
        mesh.pose.orientation.z = pose.rot.z;
        mesh.pose.orientation.w = pose.rot.w;
        char meshpath[200];
        sprintf(meshpath,"package://roboy_models/legs_with_muscles_simplified/cad/%s.STL",
                link_name.c_str() );
        mesh.mesh_resource = meshpath;
        marker_visualization_pub.publish(mesh);
    }
}

void WalkController::publishSimulationState(){
    roboy_simulation::SimulationState msg;
    msg.roboyID = roboyID;
    msg.F_contact = F_contact;
    msg.d_lift = d_lift;
    msg.d_prep = d_prep;
    msg.F_max = F_max;
    msg.psi_heading = psi_heading;
    msg.omega_heading = omega_heading;
    msg.v_forward = v_forward;
    msg.v_COM = v_COM;
    msg.k_v = k_v;
    msg.k_h = k_h;
    msg.k_p_theta_left.assign(k_p_theta_left, k_p_theta_left+4);
    msg.k_d_phi.assign(k_d_phi,k_d_phi+2);
    msg.k_p_theta_right.assign(k_p_theta_right, k_p_theta_right+4);
    msg.k_d_theta_left.assign(k_d_theta_left, k_d_theta_left+4);
    msg.k_d_theta_right.assign(k_d_theta_right, k_d_theta_right+4);
    msg.k_p_phi.assign(k_p_theta_left, k_p_theta_left+4);
    msg.k_d_phi.assign(k_p_theta_left, k_p_theta_left+4);
    msg.k_V = k_V;
    msg.k_P = k_P;
    msg.k_Q = k_Q;
    msg.k_omega = k_omega;
    msg.k_M_Fplus = k_M_Fplus;
    msg.c_hip_lift = c_hip_lift;
    msg.c_knee_lift = c_knee_lift;
    msg.c_stance_lift = c_stance_lift;
    msg.c_swing_prep = c_swing_prep;
    msg.theta_groin_0.assign(theta_groin_0, theta_groin_0+2);
    msg.phi_groin_0.assign(phi_groin_0, phi_groin_0+2);
    msg.theta_trunk_0 = theta_trunk_0;
    msg.phi_trunk_0 = phi_trunk_0;
    msg.theta_knee.assign(theta_knee, theta_knee+2);
    msg.theta_ankle.assign(theta_ankle, theta_ankle+2);
    msg.d_s.assign(d_s, d_s+2);
    msg.d_c.assign(d_c, d_c+2);
    msg.v_s.assign(v_s, v_s+2);
    msg.v_c.assign(v_c, v_c+2);
    msg.sim_time = gz_time_now.Float();
    simulation_state_pub.publish(msg);
}

void WalkController::publishID(){
    std_msgs::Int32 msg;
    msg.data = roboyID;
    roboyID_pub.publish(msg);
}

void WalkController::publishLegState(){
    roboy_simulation::LegState msgLeft;
    msgLeft.roboyID = roboyID;
    msgLeft.leg = LEG::LEFT;
    msgLeft.state = leg_state[LEG::LEFT];
    leg_state_pub.publish(msgLeft);
    roboy_simulation::LegState msgRight;
    msgRight.roboyID = roboyID;
    msgRight.leg = LEG::RIGHT;
    msgRight.state = leg_state[LEG::RIGHT];
    leg_state_pub.publish(msgRight);
}

void WalkController::publishStateMachineParameters(){
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    char momentarmnamespace[20];
    sprintf(momentarmnamespace, "statemachineparams_%d", roboyID);
    arrow.ns = momentarmnamespace;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.r = 1.0f;
    arrow.color.g = 1.0f;
    arrow.color.b = 0.0f;
    arrow.lifetime = ros::Duration();
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.03;
    arrow.scale.z = 0.03;
    arrow.action = visualization_msgs::Marker::ADD;

    visualization_msgs::Marker text;
    text.header.frame_id = "world";
    text.ns = momentarmnamespace;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    text.lifetime = ros::Duration();
    text.scale.z = 0.03;
    text.action = visualization_msgs::Marker::ADD;
    text.id = message_counter++;
    text.header.stamp = ros::Time::now();
    text.pose.orientation.w = 1.0;

    geometry_msgs::Point p;

    arrow.color.a = 0.2;
    arrow.id = message_counter++;
    arrow.header.stamp = ros::Time::now();
    arrow.points.clear();
    p.x = center_of_mass[POSITION].x;
    p.y = center_of_mass[POSITION].y;
    p.z = center_of_mass[POSITION].z;
    arrow.points.push_back(p);
    p.x = foot_sole_global[LEG::LEFT].x;
    p.y = foot_sole_global[LEG::LEFT].y;
    p.z = foot_sole_global[LEG::LEFT].z;
    arrow.points.push_back(p);
    marker_visualization_pub.publish(arrow);
    text.id = message_counter++;
    text.pose.position = p;
    text.text = "foot_sole[left]";
    marker_visualization_pub.publish(text);

    arrow.color.a = 0.2;
    arrow.id = message_counter++;
    arrow.header.stamp = ros::Time::now();
    arrow.points.clear();
    p.x = center_of_mass[POSITION].x;
    p.y = center_of_mass[POSITION].y;
    p.z = center_of_mass[POSITION].z;
    arrow.points.push_back(p);
    p.x = foot_sole_global[LEG::RIGHT].x;
    p.y = foot_sole_global[LEG::RIGHT].y;
    p.z = foot_sole_global[LEG::RIGHT].z;
    arrow.points.push_back(p);
    marker_visualization_pub.publish(arrow);
    text.id = message_counter++;
    text.pose.position = p;
    text.text = "foot_sole[right]";
    marker_visualization_pub.publish(text);

    math::Pose hip_pose = parent_model->GetLink("hip")->GetWorldPose();

    // d_s
    arrow.color.a = 1.0;
    arrow.color.r = 1.0f;
    arrow.color.g = 0.0f;
    arrow.color.b = 0.0f;
    arrow.id = message_counter++;
    arrow.header.stamp = ros::Time::now();
    arrow.points.clear();
    p.x = center_of_mass[POSITION].x;
    p.y = center_of_mass[POSITION].y;
    p.z = 0;
    arrow.points.push_back(p);
    p.x += d_s[LEG::LEFT]*hip_CS->Xn.x;
    p.y += d_s[LEG::LEFT]*hip_CS->Xn.y;
    p.z = 0;
    arrow.points.push_back(p);
    marker_visualization_pub.publish(arrow);
    text.id = message_counter++;
    text.pose.position = p;
    text.text = "d_s[left]";
    marker_visualization_pub.publish(text);

    // d_c
    arrow.color.r = 0.0f;
    arrow.color.g = 1.0f;
    arrow.color.b = 0.0f;
    arrow.id = message_counter++;
    arrow.header.stamp = ros::Time::now();
    arrow.points.clear();
    p.x = center_of_mass[POSITION].x;
    p.y = center_of_mass[POSITION].y;
    p.z = 0;
    arrow.points.push_back(p);
    p.x += d_c[LEG::LEFT]*hip_CS->Yn.x;
    p.y += d_c[LEG::LEFT]*hip_CS->Yn.y;
    p.z = 0;
    arrow.points.push_back(p);
    marker_visualization_pub.publish(arrow);
    text.id = message_counter++;
    text.pose.position = p;
    text.text = "d_c[left]";
    marker_visualization_pub.publish(text);

    // d_s
    arrow.color.a = 1.0;
    arrow.color.r = 1.0f;
    arrow.color.g = 0.0f;
    arrow.color.b = 0.0f;
    arrow.id = message_counter++;
    arrow.header.stamp = ros::Time::now();
    arrow.points.clear();
    p.x = center_of_mass[POSITION].x;
    p.y = center_of_mass[POSITION].y;
    p.z = 0;
    arrow.points.push_back(p);
    p.x += d_s[LEG::RIGHT]*hip_CS->Xn.x;
    p.y += d_s[LEG::RIGHT]*hip_CS->Xn.y;
    p.z = 0;
    arrow.points.push_back(p);
    marker_visualization_pub.publish(arrow);
    text.id = message_counter++;
    text.pose.position = p;
    text.text = "d_s[right]";
    marker_visualization_pub.publish(text);

    // d_c
    arrow.color.r = 0.0f;
    arrow.color.g = 1.0f;
    arrow.color.b = 0.0f;
    arrow.id = message_counter++;
    arrow.header.stamp = ros::Time::now();
    arrow.points.clear();
    p.x = center_of_mass[POSITION].x;
    p.y = center_of_mass[POSITION].y;
    p.z = 0;
    arrow.points.push_back(p);
    p.x += d_c[LEG::RIGHT]*hip_CS->Yn.x;
    p.y += d_c[LEG::RIGHT]*hip_CS->Yn.y;
    p.z = 0;
    arrow.points.push_back(p);
    marker_visualization_pub.publish(arrow);
    text.id = message_counter++;
    text.pose.position = p;
    text.text = "d_c[right]";
    marker_visualization_pub.publish(text);
}

void WalkController::publishCoordinateSystems(physics::LinkPtr parent_link, ros::Time time, bool child_link){
    tf::Transform tf0, tf1;
    math::Pose pose = parent_link->GetRelativePose();
    tf0.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
    tf0.setRotation(tf::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w));
    if(!child_link) { // parent_link is top link and connected to world frame
        tf_broadcaster.sendTransform(tf::StampedTransform(tf0, time, "world", parent_link->GetName()));
    }
    physics::Link_V child_links = parent_link->GetChildJointsLinks();
    if (child_links.empty()) {
        return;
    } else {
        for (auto child_link:child_links) { // each child relative pose to parent
            math::Pose pose = child_link->GetRelativePose();
            tf1.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
            tf1.setRotation(tf::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w));
            tf1 = tf0.inverseTimes(tf1);
            tf_broadcaster.sendTransform(tf::StampedTransform(tf1, time, parent_link->GetName(), child_link->GetName()));
            publishCoordinateSystems(child_link, time, true);
        }
    }
}

void WalkController::toggleWalkController(const std_msgs::Bool::ConstPtr &msg){
    control = msg->data;
}

GZ_REGISTER_MODEL_PLUGIN(WalkController)