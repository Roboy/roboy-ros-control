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
        roboy_visualization_control_sub = nh->subscribe("/roboy/visualization_control", 10, &RoboySim::visualization_control, this);

        visualizeTendon_pub = nh->advertise<roboy_simulation::Tendon>("/visual/tendon", 1);
        marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);

        cmd = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
        pos = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
        vel = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];
        eff = new double[NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION];

        force_torque_ankle_left_sub  = nh->subscribe("/roboy/force_torque_ankle_left", 1,
                                                     &RoboySim::finite_state_machine, this);
        force_torque_ankle_right_sub  = nh->subscribe("/roboy/force_torque_ankle_right", 1,
                                                      &RoboySim::finite_state_machine, this);
    }

    RoboySim::~RoboySim() {
        delete nh;
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
                ROS_INFO("Loading Muscle Plugin");
                sim_muscles.push_back(class_loader->createInstance("roboy_simulation::IMuscle"));
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
                       robot_namespace.c_str());;
//            initializeControllerParameters(params, model.back());
//            controllerParams.push_back(params);
//            roboy_simulation::ControllerParameters msg;
//            msg.roboyID = i;
//            controllerParametersToMessage(params, msg);
//            control_parameters_pub.publish(msg);

        ROS_INFO_NAMED("gazebo_ros_control", "Parsing myoMuscles");
        if (!parseMyoMuscleSDF(sdf_->ToString(""), myoMuscles))
            ROS_WARN_NAMED("gazebo_ros_control", "ERROR parsing myoMuscles, check your sdf file.");
        numberOfMyoMuscles = myoMuscles.size();
        ROS_INFO("Found %d MyoMuscles in sdf file", numberOfMyoMuscles);

        // class laoder for loading muscle plugins
        class_loader.reset(new pluginlib::ClassLoader<roboy_simulation::IMuscle>
                                   ("roboy_simulation",
                                    "roboy_simulation::IMuscle"));

        // Listen to the update event. This event is broadcast every simulation iteration.
        update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&RoboySim::Update, this));

        ROS_INFO_NAMED("gazebo_ros_control", "Loaded gazebo_ros_control.");
    }

    void RoboySim::readSim(ros::Time time, ros::Duration period) {
        ROS_DEBUG("read simulation");
        // update muscle plugins
        for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
            for(int i = 0; i < sim_muscles[muscle]->viaPoints.size(); i++){
                math::Pose linkPose = sim_muscles[muscle]->viaPoints[i]->link->GetWorldPose();
                sim_muscles[muscle]->viaPoints[i]->linkPosition = linkPose.pos;
                sim_muscles[muscle]->viaPoints[i]->linkRotation = linkPose.rot;
            }
            sim_muscles[muscle]->Update(time, period);
        }
        if(visualizeTendon)
            publishTendon();
        if(visualizeForce)
            publishForce();
        if(visualizeCOM)
            publishCOM();

    }

    void RoboySim::writeSim(ros::Time time, ros::Duration period) {
        ROS_DEBUG("write simulation");
        // apply the calculated forces
        for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
            for(int i = 0; i < sim_muscles[muscle]->viaPoints.size(); i++){
                std::shared_ptr<roboy_simulation::IViaPoints> vp = sim_muscles[muscle]->viaPoints[i];
                vp->link->AddForceAtWorldPosition(vp->prevForce, vp->prevForcePoint);
                vp->link->AddForceAtWorldPosition(vp->nextForce, vp->nextForcePoint);
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

    void RoboySim::visualization_control(const roboy_simulation::VisualizationControl::ConstPtr &msg){
        switch (msg->control){
            case Tendon:{
                visualizeTendon = msg->value;
                break;
            }
            case COM:{
                visualizeCOM = msg->value;
                break;
            }
            case Force:{
                visualizeForce = msg->value;
                break;
            }
        }
    }

    void RoboySim::publishTendon(){
        ROS_INFO_THROTTLE(1.0,"publish tendon");
        static bool add = true;
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "world";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "tendon";
        if(add) {
            line_strip.action = visualization_msgs::Marker::ADD;
            add = false;
        }else{
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
            line_strip.id = 1000+muscle;
            for (uint i = 0; i < sim_muscles[muscle]->viaPoints.size(); i++) {
                geometry_msgs::Vector3 v;
                std::shared_ptr<roboy_simulation::IViaPoints> vp = sim_muscles[muscle]->viaPoints[i];
                v.x = vp->prevForcePoint.x;
                v.y = vp->prevForcePoint.y;
                v.z = vp->prevForcePoint.z;
                msg.viaPoints.push_back(v);
                v.x = vp->nextForcePoint.x;
                v.y = vp->nextForcePoint.y;
                v.z = vp->nextForcePoint.z;
                msg.viaPoints.push_back(v);
                geometry_msgs::Point p;
                p.x = vp->prevForcePoint.x;
                p.y = vp->prevForcePoint.y;
                p.z = vp->prevForcePoint.z;
                line_strip.points.push_back(p);
                p.x = vp->nextForcePoint.x;
                p.y = vp->nextForcePoint.y;
                p.z = vp->nextForcePoint.z;
                line_strip.points.push_back(p);
            }
            marker_visualization_pub.publish(line_strip);
        }
        visualizeTendon_pub.publish(msg);
    }

    void RoboySim::publishCOM(){
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
        if(add) {
            sphere.action = visualization_msgs::Marker::ADD;
            add = false;
        }else{
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

    void RoboySim::publishForce(){
        static bool add = true;
        visualization_msgs::Marker arrow;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        arrow.header.frame_id = "world";
        arrow.ns = "force";
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.color.r = 0.0f;
        arrow.color.g = 1.0f;
        arrow.color.b = 0.0f;
        arrow.color.a = 1.0;
        arrow.lifetime = ros::Duration();
        arrow.scale.x = 0.005;
        arrow.scale.y = 0.005;
        arrow.scale.z = 0.1;
        arrow.id = 1002;
        if(add) {
            arrow.action = visualization_msgs::Marker::ADD;
            add = false;
        }else{
            arrow.action = visualization_msgs::Marker::MODIFY;
        }
        uint id = 0;
        for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
            for (uint i = 0; i < sim_muscles[muscle]->viaPoints.size(); i++) {
                arrow.id = id;
                if (fabs(sim_muscles[muscle]->viaPoints[i]->prevForce.GetSquaredLength()) > 0.0) {
                    arrow.header.stamp = ros::Time::now();
                    arrow.points.clear();
                    geometry_msgs::Point p;
                    p.x = sim_muscles[muscle]->viaPoints[i]->prevForcePoint.x;
                    p.y = sim_muscles[muscle]->viaPoints[i]->prevForcePoint.y;
                    p.z = sim_muscles[muscle]->viaPoints[i]->prevForcePoint.z;
                    arrow.points.push_back(p);
                    p.x += sim_muscles[muscle]->viaPoints[i]->prevForce.x;
                    p.y += sim_muscles[muscle]->viaPoints[i]->prevForce.y;
                    p.z += sim_muscles[muscle]->viaPoints[i]->prevForce.z;
                    arrow.points.push_back(p);
                } else {
                    arrow.action = visualization_msgs::Marker::DELETE;
                    add = true;
                }
                marker_visualization_pub.publish(arrow);
                if (fabs(sim_muscles[muscle]->viaPoints[i]->nextForce.GetSquaredLength()) > 0.0) {
                    arrow.header.stamp = ros::Time::now();
                    arrow.points.clear();
                    geometry_msgs::Point p;
                    p.x = sim_muscles[muscle]->viaPoints[i]->nextForcePoint.x;
                    p.y = sim_muscles[muscle]->viaPoints[i]->nextForcePoint.y;
                    p.z = sim_muscles[muscle]->viaPoints[i]->nextForcePoint.z;
                    arrow.points.push_back(p);
                    p.x += sim_muscles[muscle]->viaPoints[i]->nextForce.x;
                    p.y += sim_muscles[muscle]->viaPoints[i]->nextForce.y;
                    p.z += sim_muscles[muscle]->viaPoints[i]->nextForce.z;
                    arrow.points.push_back(p);
                } else {
                    arrow.action = visualization_msgs::Marker::DELETE;
                    add = true;
                }
                marker_visualization_pub.publish(arrow);
            }
        }
    }

    void RoboySim::calculateCOM(int type, math::Vector3 &COM) {
        physics::Link_V links = parent_model->GetLinks();
        double mass_total = 0;
        COM = math::Vector3(0, 0, 0);
        for (auto link:links) {
            if(link->GetName().compare("halterung")==0)
                continue;
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
    vector<double> RoboySim::calculateAngle_links(vector<pair<std::string, std::string>> _linkpair, int flag){
        vector<double> angle;
        for(auto compare : _linkpair) {
            physics::LinkPtr link1;
            link1 = parent_model->GetLink(compare.first);
            physics::LinkPtr link2;
            link2 = parent_model->GetLink(compare.second);
            math::Pose p1= link1->GetWorldCoGPose();
            math::Pose p2= link2->GetWorldCoGPose();
            math::Vector3 Euler1=p1.rot.GetAsEuler();
            math::Vector3 Euler2=p2.rot.GetAsEuler();
            switch (flag){
                case 1: angle.push_back(Euler1.y-Euler2.y);      /** sagittal */
                    break;
                case 2: angle.push_back(Euler1.x-Euler2.x);      /** coronal */
                    break;
                case 3: angle.push_back(Euler1.z-Euler2.z);      /** traversal */
                    break;
            }
        }
        return angle;
    }
    map<string,math::Vector3> RoboySim::calculateTrunk(){
        physics::LinkPtr trunk = parent_model->GetLink("hip");
        math::Pose p = trunk->GetWorldCoGPose();
        math::Vector3 Euler = p.rot.GetAsEuler();
        math::Vector3 velocity = trunk->GetWorldCoGLinearVel();
        map<string,math::Vector3> r;
        string a("Velocity");
        string b("Angle");
        r.insert({a,velocity});
        r.insert({b,Euler});
        return r;
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
                for (link_child_it = myoMuscle_it->FirstChildElement("link"); link_child_it;
                     link_child_it = link_child_it->NextSiblingElement("link")) {
                    string linkname = link_child_it->Attribute("name");
                    physics::LinkPtr link = parent_model->GetLink(linkname);
                    if ((!linkname.empty()) && link) {
                        TiXmlElement *viaPoint_child_it = NULL;
                        for (viaPoint_child_it = link_child_it->FirstChildElement("viaPoint"); viaPoint_child_it;
                             viaPoint_child_it = viaPoint_child_it->NextSiblingElement("viaPoint")) {
                            roboy_simulation::ViaPointInfo vp;
                            vp.link = link;
                            float x, y, z;
                            if (sscanf(viaPoint_child_it->GetText(), "%f %f %f", &x, &y, &z) != 3) {
                                ROS_ERROR_STREAM_NAMED("parser", "error reading [via point] (x y z)");
                                return false;
                            }
                            vp.point = math::Vector3(x,y,z);
                            if (viaPoint_child_it->Attribute("type")){
                                string type = viaPoint_child_it->Attribute("type");
                                if (type == "FIXPOINT") {
                                    vp.type = roboy_simulation::IViaPoints::FIXPOINT;
                                } else if (type == "SPHERICAL" || type == "CYLINDRICAL") {
                                    if (viaPoint_child_it->QueryDoubleAttribute("radius", &vp.radius) != TIXML_SUCCESS){
                                        ROS_ERROR_STREAM_NAMED("parser", "error reading radius");
                                        return false;
                                    }
                                    if (viaPoint_child_it->QueryIntAttribute("state", &vp.state) != TIXML_SUCCESS){
                                        ROS_ERROR_STREAM_NAMED("parser", "error reading state");
                                        return false;
                                    }
                                    if (viaPoint_child_it->QueryIntAttribute("revCounter", &vp.revCounter) != TIXML_SUCCESS){
                                        ROS_ERROR_STREAM_NAMED("parser", "error reading revCounter");
                                        return false;
                                    }
                                    if (type == "SPHERICAL") {
                                        vp.type = roboy_simulation::IViaPoints::SPHERICAL;
                                    } else {
                                        vp.type = roboy_simulation::IViaPoints::CYLINDRICAL;
                                    }
                                } else if (type == "MESH") {
                                    // TODO
                                } else {
                                    ROS_ERROR_STREAM_NAMED("parser", "unknown type of via point: " + type);
                                    return false;
                                }
                            } else {
                                ROS_ERROR_STREAM_NAMED("parser", "error reading type");
                                return false;
                            }
                            myoMuscle.viaPoints.push_back(vp);
                        }
                        if (myoMuscle.viaPoints.empty()) {
                            ROS_ERROR_STREAM_NAMED("parser", "No viaPoint element found in myoMuscle '"
                                    << myoMuscle.name << "' link element.");
                            return false;
                        }
                    } else {
                        ROS_ERROR_STREAM_NAMED("parser", "No link name attribute specified for myoMuscle'"
                                << myoMuscle.name << "'.");
                        continue;
                    }
                }
//                uint j = 0;
//                for (uint i = 0; i < myoMuscle.viaPoints.size(); i++) {
//                    // absolute position + relative position=actual position of each via point
//                    ROS_INFO("%s: %f %f %f", myoMuscle.links[j]->GetName().c_str(),
//                             myoMuscle.viaPoints[i].x, myoMuscle.viaPoints[i].y, myoMuscle.viaPoints[i].z);
//                    if(i>=myoMuscle.link_index[j]-1){
//                        j++;
//                    }
//                }
                ROS_INFO("%ld viaPoints for myoMuscle %s", myoMuscle.viaPoints.size(), myoMuscle.name.c_str() );

                //check if wrapping surfaces are enclosed by fixpoints
                for(int i = 0; i < myoMuscle.viaPoints.size(); i++){
                    if(i == 0 && myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT){
                        ROS_ERROR_STREAM_NAMED("parser", "muscle insertion has to be a fix point");
                        return false;
                    }
                    if(i == myoMuscle.viaPoints.size()-1 && myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT){
                        ROS_ERROR_STREAM_NAMED("parser", "muscle fixation has to be a fix point");
                        return false;
                    }
                    if(myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT){
                        if(myoMuscle.viaPoints[i-1].type != roboy_simulation::IViaPoints::FIXPOINT
                            || myoMuscle.viaPoints[i+1].type != roboy_simulation::IViaPoints::FIXPOINT){
                            ROS_ERROR_STREAM_NAMED("parser", "non-FIXPOINT via-points have to be enclosed by two FIXPOINT via-points");
                            return false;
                        }
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

    void RoboySim::finite_state_machine(const roboy_simulation::ForceTorque::ConstPtr &msg){
        // check what state the leg is currently in
        LEG_STATE leg_state;
        math::Vector3 foot_pos;

        if(msg->leg == LEG::LEFT){
            leg_state = left_leg_state;
            foot_pos = parent_model->GetLink("foot_left")->GetWorldCoGPose().pos;
        }else if(msg->leg == LEG::RIGHT){
            leg_state = right_leg_state;
            foot_pos = parent_model->GetLink("foot_right")->GetWorldCoGPose().pos;
        }

        bool state_transition = false;

        switch(leg_state){
            case Stance:{
                // calculate the COM
                math::Vector3 COM;
                calculateCOM(POSITION, COM);
                // calculate signed horizontal distance between foot_pos and COM
                double d_s = sqrt(pow(foot_pos.x-COM.x,2.0)+pow(foot_pos.y-COM.y,2.0)) *
                             ((foot_pos.x - COM.x ) < 0 && (foot_pos.y - COM.y ) < 0)? -1.0 : 1.0;
                if(d_s < d_lift || (left_leg_state == Stance && right_leg_state == Stance)){
                    state_transition = true;
                }
                break;
            }
            case Lift_off:{
                double force_norm = sqrt(pow(msg->force.x, 2.0)+pow(msg->force.y, 2.0)+pow(msg->force.z, 2.0));
                if(force_norm < F_contact){
                    state_transition = true;
                }
                break;
            }
            case Swing:{
                // calculate the COM
                math::Vector3 COM;
                calculateCOM(POSITION, COM);
                // calculate signed horizontal distance between foot_pos and COM
                double d_s = sqrt(pow(foot_pos.x-COM.x,2.0)+pow(foot_pos.y-COM.y,2.0)) *
                             ((foot_pos.x - COM.x ) < 0 && (foot_pos.y - COM.y ) < 0)? -1.0 : 1.0;
                if(d_s > d_prep){
                    state_transition = true;
                }
                break;
            }
            case Stance_Preparation:{
                double force_norm = sqrt(pow(msg->force.x, 2.0)+pow(msg->force.y, 2.0)+pow(msg->force.z, 2.0));
                if(force_norm > F_contact){
                    state_transition = true;
                }
                break;
            }
        }

        if(state_transition) {
            if (msg->leg == LEG::LEFT) {
                left_leg_state = NextState(left_leg_state);
            } else if (msg->leg == LEG::RIGHT) {
                right_leg_state = NextState(right_leg_state);
            }
        }
    }
}

GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::RoboySim)
