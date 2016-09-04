#include "walkController.hpp"

WalkController::WalkController() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "WalkController",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = new ros::NodeHandle;

    force_torque_ankle_left_sub = nh->subscribe("/roboy/force_torque_ankle_left", 1,
                                                &WalkController::finite_state_machine, this);
    force_torque_ankle_right_sub = nh->subscribe("/roboy/force_torque_ankle_right", 1,
                                                 &WalkController::finite_state_machine, this);

    roboy_visualization_control_sub = nh->subscribe("/roboy/visualization_control", 10,
                                                    &WalkController::visualization_control, this);

    visualizeTendon_pub = nh->advertise<roboy_simulation::Tendon>("/visual/tendon", 1);
    marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // the following links are part of my robot (this is useful if the model.sdf contains additional links)
    link_names.push_back("hip");
    link_names.push_back("thigh_left");
    link_names.push_back("thigh_right");
    link_names.push_back("shank_left");
    link_names.push_back("shank_right");
    link_names.push_back("foot_left");
    link_names.push_back("foot_right");

    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        muscles_spanning_joint[sim_muscles[muscle]->joint->GetName()].push_back(muscle);
    }

    leg_state[LEG::LEFT] = Stance;
    leg_state[LEG::RIGHT] = Swing;

    k_v = 1.0;
    k_h = 1.0;
    k_p_theta_left[0] = 1.0;
    k_p_theta_left[1] = 1.0;
    k_p_theta_left[2] = 1.0;
    k_p_theta_left[3] = 1.0;
    k_p_theta_right[0] = 1.0;
    k_p_theta_right[1] = 1.0;
    k_p_theta_right[2] = 1.0;
    k_p_theta_right[3] = 1.0;
    k_d_theta_left[0] = 1.0;
    k_d_theta_left[1] = 1.0;
    k_d_theta_left[2] = 1.0;
    k_d_theta_left[3] = 1.0;
    k_d_theta_right[0] = 1.0;
    k_d_theta_right[1] = 1.0;
    k_d_theta_right[2] = 1.0;
    k_d_theta_right[3] = 1.0;
    k_p_phi[0] = 1.0;
    k_p_phi[1] = 1.0;
    k_d_phi[0] = 1.0;
    k_d_phi[1] = 1.0;
    // target force torque gains
    k_V = 1.0;
    k_P = 1.0;
    k_Q = 1.0;
    k_omega = 1.0;
}

WalkController::~WalkController() {
    delete nh;
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

    // Get the Gazebo simulation period
    ros::Duration gazebo_period(parent_model->GetWorld()->GetPhysicsEngine()->GetMaxStepSize());

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
            sim_muscles.back()->Init(myoMuscles[muscle]);
        }
        catch (pluginlib::PluginlibException &ex) {
            //handle the class failing to load
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
    }

    updateFootDisplacementAndVelocity();

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&WalkController::Update, this));

    ROS_INFO("WalkController ready");
}

void WalkController::Update() {
    // Get the simulation time and period
    gazebo::common::Time gz_time_now = parent_model->GetWorld()->GetSimTime();
    ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
    ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros;

    // Check if we should update the controllers
    if (sim_period >= control_period) {
        // Store this simulation time
        last_update_sim_time_ros = sim_time_ros;

        // Update the robot simulation with the state of the gazebo model
        readSim(sim_time_ros, sim_period);

        // Compute the controller commands
        if(updateTargetFeatures());
            updateMuscleActivity();
    }

    // Update the gazebo model with the result of the controller
    // computation
    writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros);
    last_write_sim_time_ros = sim_time_ros;
    ros::spinOnce();
}

void WalkController::readSim(ros::Time time, ros::Duration period) {
    ROS_DEBUG("read simulation");
    // update muscle plugins
    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        sim_muscles[muscle]->Update(time, period);
    }

    if (visualizeTendon)
        publishTendon();
    if (visualizeForce)
        publishForce();
    if (visualizeCOM)
        publishCOM();
    if (visualizeMomentArm)
        publishMomentArm();
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

bool WalkController::updateFootDisplacementAndVelocity(){
    // calculate the COM position and velocity
    calculateCOM(POSITION, center_of_mass[POSITION]);
    calculateCOM(VELOCITY, center_of_mass[VELOCITY]);

    // calculate signed horizontal distance between foot_pos and COM
    math::Pose hip_pose = parent_model->GetLink("hip")->GetWorldPose();

    // get the stance leg
    LEG stance_leg = getLegInState(Stance);

    if(stance_leg == LEG::NONE){
        return false;
    }

    // get the foot position and velocity
    math::Vector3 foot_pos = parent_model->GetLink(FOOT[stance_leg])->GetWorldPose().pos;
    math::Vector3 foot_vel = parent_model->GetLink(FOOT[stance_leg])->GetWorldCoGLinearVel();

    // calculate the distance and velocity between foot and center of mass in world frame
    math::Vector3 d_foot_pos = foot_pos - center_of_mass[POSITION];
    math::Vector3 d_foot_vel = foot_vel - center_of_mass[VELOCITY];

    // rotate into hip frame
    d_foot_pos = hip_pose.rot.RotateVector(d_foot_pos);
    d_foot_vel = hip_pose.rot.RotateVector(d_foot_vel);
    v_COM = hip_pose.rot.RotateVector(hip_pose.pos).x;

    // calculate signed sagittal and coronal foot displacement and velocity
    d_s = d_foot_pos.x;
    d_c = d_foot_pos.y;
    v_s = d_foot_vel.x;
    v_c = d_foot_vel.y;

    return true;
}

void WalkController::finite_state_machine(const roboy_simulation::ForceTorque::ConstPtr &msg) {
    // check what state the leg is currently in
    bool state_transition = false;

    switch (leg_state[msg->leg]) {
        case Stance: {
            if (d_s < d_lift || (leg_state[LEG::LEFT] == Stance && leg_state[LEG::RIGHT] == Stance)) {
                state_transition = true;
                ROS_INFO("ds: %f d_lift: %f", d_s, d_lift);
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
            if (d_s > d_prep) {
                state_transition = true;
                ROS_INFO("d_s: %f d_prep: %f", d_s, d_prep);
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

bool WalkController::updateTargetFeatures() {
    if(!updateFootDisplacementAndVelocity()){
        ROS_WARN_THROTTLE(1.0, "legs are no touching the ground");
        return false;
    }

    // target velocity
    math::Quaternion heading(0, 0, psi_heading);
    math::Vector3 v_target(v_forward, 0, 0);
    v_target = heading.RotateVector(v_target);

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
    double theta_groin_left = theta_groin_0[LEG::LEFT] + k_p_theta_left[leg_state[LEG::LEFT]] * d_s
                              + k_d_theta_left[leg_state[LEG::LEFT]] * (v_forward - v_s);
    double phi_groin_left = phi_groin_0[LEG::LEFT] + k_p_phi[LEG::LEFT] * d_c - k_d_phi[LEG::LEFT] * v_c;
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
    double theta_groin_right = theta_groin_0[LEG::RIGHT] + k_p_theta_right[leg_state[LEG::RIGHT]] * d_s
                               + k_d_theta_right[leg_state[LEG::RIGHT]] * (v_forward - v_s);
    double phi_groin_right = phi_groin_0[LEG::RIGHT] + k_p_phi[LEG::RIGHT] * d_c - k_d_phi[LEG::RIGHT] * v_c;
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

    return true;
}

void WalkController::updateMuscleActivity() {
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
                math::Vector3 momentArm_normalized = momentArm.Normalize();
                tau[sim_muscles[muscle]->name] =
                        momentArm_normalized.Cross(link->GetWorldPose().pos - joint->GetWorldPose().pos)
                                .Dot(F[link_name]) + momentArm_normalized.Dot(T[link_name]);
                F_tilde[sim_muscles[muscle]->name] = tau[sim_muscles[muscle]->name] / momentArm.GetLength();
                sim_muscles[muscle]->cmd = F_tilde[sim_muscles[muscle]->name];
                ROS_INFO("")
            }
        }
    }
}

void WalkController::visualization_control(const roboy_simulation::VisualizationControl::ConstPtr &msg) {
    switch (msg->control) {
        case Tendon: {
            visualizeTendon = msg->value;
            break;
        }
        case COM: {
            visualizeCOM = msg->value;
            break;
        }
        case Force: {
            visualizeForce = msg->value;
            break;
        }
        case MomentArm: {
            visualizeMomentArm = msg->value;
            break;
        }
    }
}

void WalkController::publishTendon() {
    static bool add = true;
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "tendon";
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
        line_strip.id = 1000 + muscle;
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
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    sphere.header.frame_id = "world";
    sphere.ns = "force";
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
    sphere.id = 1001;
    math::Vector3 comPosition;
    calculateCOM(POSITION, comPosition);
    sphere.pose.position.x = comPosition.x;
    sphere.pose.position.y = comPosition.y;
    sphere.pose.position.z = comPosition.z;
    marker_visualization_pub.publish(sphere);
}

void WalkController::publishForce() {
    uint id = 1000;
    visualization_msgs::Marker arrow;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    arrow.header.frame_id = "world";
    arrow.ns = "force";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration();
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.005;
    arrow.scale.z = 0.1;
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
    uint id = 100000;
    visualization_msgs::Marker arrow;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    arrow.header.frame_id = "world";
    arrow.ns = "momentArm";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.r = 0.0f;
    arrow.color.g = 0.0f;
    arrow.color.b = 1.0f;
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration();
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.005;
    arrow.scale.z = 0.1;
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

GZ_REGISTER_MODEL_PLUGIN(WalkController)