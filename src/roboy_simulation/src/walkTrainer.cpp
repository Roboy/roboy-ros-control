#include <thread>
#include "walkTrainer.hpp"

WalkTrainer::WalkTrainer(FitFunc &func, CMAParameters<> &parameters):CMAStrategy<CovarianceUpdate>(func,parameters) {
    // Create a new transport node
    node = transport::NodePtr(new transport::Node());

    // Initialize the node with the world name
    node->Init("walkTrainer");

    // Create a publisher on the ~/physics topic
    transport::PublisherPtr physicsPub = node->Advertise<msgs::Physics>("~/physics");

    // set the physic params
    msgs::Physics physicsMsg;
    physicsMsg.set_type(msgs::Physics::ODE);
    physicsMsg.set_max_step_size(0.0003);
    physicsPub->Publish(physicsMsg);

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "WalkTrainer", ros::init_options::NoSigintHandler);
    }

    resetPub = node->Advertise<gazebo::msgs::WorldControl>("/gazebo/default/world_control");

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    reset_world_srv = nh->advertiseService("/roboy/reset_world", &WalkTrainer::resetWorld, this);
    sim_control_sub = nh->subscribe("/roboy/sim_control", 1, &WalkTrainer::simulationControl, this);

    initializeWorlds(15);

    controllerParams.resize(15);
};

WalkTrainer::~WalkTrainer() {
}

void WalkTrainer::initializeWorlds(uint numberOfWorlds) {
    // load numberOfWorlds empty worlds
    for (uint i = 0; i < numberOfWorlds; i++) {
        world.push_back(gazebo::loadWorld("worlds/empty.world"));
    }
    gazebo::sensors::run_once(true);
    gazebo::sensors::run_threads();

    // load the legs in each world
    for (uint i = 0; i < numberOfWorlds; i++) {
        world[i]->InsertModelFile("model://legs_with_muscles_simplified");

        // wait until the model is loaded
        int modelCountBefore = world[i]->GetModelCount();
        int retry = 0;
        while (world[i]->GetModelCount() == modelCountBefore) {
            gazebo::runWorld(world[i], 100);
            gazebo::common::Time::MSleep(100);
            retry++;
            if (retry > 1000)
                break;
        }
        if (world[i]->GetModelCount() == modelCountBefore + 1) {
            ROS_INFO("Successfully inserted model");
            model.push_back(world[i]->GetModel("legs_with_muscles_simplified"));
            roboyIDs.push_back(i);
            char topic[200];
            sprintf(topic, "/roboy%d/controller_parameters", i);
            control_parameters_srvs.push_back( nh->serviceClient<roboy_simulation::UpdateControllerParameters>(topic));
            sprintf(topic, "/roboy%d/energies", i);
            energie_srvs.push_back( nh->serviceClient<roboy_simulation::Energies>(topic));
        } else {
            ROS_WARN("Failed inserting model");
        }
    }
}

void WalkTrainer::simulate() {
    for (uint i = 0; i < world.size(); i++) {
        if (!paused) {
            gazebo::sensors::run_once(true);
            if (slow_motion) {
                gazebo::runWorld(world[i], 1);
                ros::Duration d(0.05);
                d.sleep();
            } else {
                gazebo::runWorld(world[i], 100);
            }
        }
    }
}

bool WalkTrainer::resetWorld(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    gazebo::msgs::WorldControl w_ctrl;
    w_ctrl.mutable_reset()->set_all(true);
    resetPub->Publish(w_ctrl);
    res.success = true;
    res.message = "resetting worlds";
    return true;
}

void WalkTrainer::initializeControllerParameters(ControllerParameters &params, physics::ModelPtr parent_model) {
    math::Vector3 euler = parent_model->GetLink("hip")->GetWorldPose().rot.GetAsEuler();
    params[phi_trunk_0] = euler.x;
    params[theta_trunk_0] = euler.y;

    euler = parent_model->GetLink("thigh_left")->GetWorldPose().rot.GetAsEuler();
    params[phi_groin_0 + LEG::LEFT] = euler.x;
    params[theta_groin_0 + LEG::LEFT] = euler.y;

    euler = parent_model->GetLink("thigh_right")->GetWorldPose().rot.GetAsEuler();
    params[phi_groin_0 + LEG::RIGHT] = euler.x;
    params[theta_groin_0 + LEG::RIGHT] = euler.y;

    params[k_v] = 0.1;
    params[k_h] = 0.1;
    params[k_p_theta_left + Stance] = 0.1;
    params[k_p_theta_left + Lift_off] = 0.1;
    params[k_p_theta_left + Swing] = 0.1;
    params[k_p_theta_left + Stance_Preparation] = 0.1;
    params[k_p_theta_right + Stance] = 0.1;
    params[k_p_theta_right + Lift_off] = 0.1;
    params[k_p_theta_right + Swing] = 0.1;
    params[k_p_theta_right + Stance_Preparation] = 0.1;
    params[k_d_theta_left + Stance] = 0.1;
    params[k_d_theta_left + Lift_off] = 0.1;
    params[k_d_theta_left + Swing] = 0.1;
    params[k_d_theta_left + Stance_Preparation] = 0.1;
    params[k_d_theta_right + Stance] = 0.1;
    params[k_d_theta_right + Lift_off] = 0.1;
    params[k_d_theta_right + Swing] = 0.1;
    params[k_d_theta_right + Stance_Preparation] = 0.1;
    params[k_p_phi + LEG::LEFT] = 0.1;
    params[k_p_phi + LEG::RIGHT] = 0.1;
    params[k_d_phi + LEG::LEFT] = 0.1;
    params[k_d_phi + LEG::RIGHT] = 0.1;
    // target force torque gains
    params[k_V] = 0.1;
    params[k_P] = 0.1;
    params[k_Q] = 0.1;
    params[k_omega] = 0.1;
}

void WalkTrainer::initializeInterActiveMarkers(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                               physics::ModelPtr model, int roboyID) {
    paused = false;
    modelControl = model;
    for (auto link:model->GetLinks()) {
        visualization_msgs::InteractiveMarker int_marker;
        string link_name = link->GetName();
        int_marker.header.frame_id = "world";
        int_marker.header.stamp = ros::Time::now();
        math::Pose pose = link->GetWorldPose();
        int_marker.pose.position.x = pose.pos.x;
        int_marker.pose.position.y = pose.pos.y;
        int_marker.pose.position.z = pose.pos.z;
        int_marker.scale = 0.1;
        int_marker.name = link_name;
        int_marker.description = link_name;

        visualization_msgs::InteractiveMarkerControl control;

        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        server->insert(int_marker);
        server->setCallback(int_marker.name, &processFeedback);
        server->applyChanges();
    }
}

dMat WalkTrainer::ask() {
    return CMAStrategy<CovarianceUpdate>::ask();
}

void WalkTrainer::eval(const dMat &candidates,
                       const dMat &phenocandidates) {
    // set the controller parameters of each roboy instantiation
    for (int roboyID = 0; roboyID < candidates.cols(); roboyID++) {
        // get the controller parameters
        _solutions.get_candidate(roboyID).set_x(candidates.col(roboyID));
        roboy_simulation::UpdateControllerParameters msg;
        controllerParams[roboyID] = _solutions.get_candidate(roboyID).get_x();
        controllerParametersToMessage(controllerParams[roboyID], msg.request.params);
        // send them to the roboy instance
        control_parameters_srvs[roboyID].call(msg);
        // update the world
        gazebo::runWorld(world[roboyID], 1);
        // retrieve energies
        roboy_simulation::Energies msg2;
        energie_srvs[roboyID].call(msg2);
        double total_energie = msg2.response.E_speed + msg2.response.E_headori + msg2.response.E_effort;
        _solutions.get_candidate(roboyID).set_fvalue(total_energie);
        //std::cerr << "candidate x: " << _solutions.get_candidate(r).get_x_dvec().transpose() << std::endl;
    }
    update_fevals(candidates.cols());
}

void WalkTrainer::tell() {
    CMAStrategy<CovarianceUpdate>::tell();
}

bool WalkTrainer::stop() {
    return CMAStrategy<CovarianceUpdate>::stop();
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
        math::Quaternion q(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
                           feedback->pose.orientation.z);
        math::Vector3 pos(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
        math::Pose pose(pos, q);
        modelControl->GetLink(feedback->marker_name)->SetWorldPose(pose);
    }
    // update all markers except for the one we are currently modifying
    for (auto link:modelControl->GetLinks()) {
        if (link->GetName() == feedback->marker_name)
            continue;
        visualization_msgs::InteractiveMarker marker;
        interactive_marker_server->get(link->GetName(), marker);
        math::Pose pose = link->GetWorldPose();
        marker.pose.position.x = pose.pos.x;
        marker.pose.position.y = pose.pos.y;
        marker.pose.position.z = pose.pos.z;
        marker.pose.orientation.w = pose.rot.w;
        marker.pose.orientation.x = pose.rot.x;
        marker.pose.orientation.y = pose.rot.y;
        marker.pose.orientation.z = pose.rot.z;
        interactive_marker_server->setPose(marker.name, marker.pose);
    }
    interactive_marker_server->applyChanges();
}

void WalkTrainer::simulationControl(const std_msgs::Int32::ConstPtr &msg) {
    switch (msg->data) {
        case Play: {
            paused = false;
            slow_motion = false;
            break;
        }
        case Pause: {
            paused = true;
            break;
        }
        case Rewind: {
            gazebo::msgs::WorldControl w_ctrl;
            w_ctrl.mutable_reset()->set_all(true);
            resetPub->Publish(w_ctrl);
            break;
        }
        case Slow_Motion: {
            slow_motion = true;
            break;
        }
        case UpdateInteractiveMarker: {
            for (auto link:modelControl->GetLinks()) {
                visualization_msgs::InteractiveMarker marker;
                interactive_marker_server->get(link->GetName(), marker);
                math::Pose pose = link->GetWorldPose();
                marker.pose.position.x = pose.pos.x;
                marker.pose.position.y = pose.pos.y;
                marker.pose.position.z = pose.pos.z;
                marker.pose.orientation.w = pose.rot.w;
                marker.pose.orientation.x = pose.rot.x;
                marker.pose.orientation.y = pose.rot.y;
                marker.pose.orientation.z = pose.rot.z;
                interactive_marker_server->setPose(marker.name, marker.pose);
            }
            interactive_marker_server->applyChanges();
            break;
        }
    }
}

int main(int _argc, char **_argv) {
    // setup Gazebo server
    if (gazebo::setupServer()) {
        std::cout << "Gazebo server setup successful\n";
    } else {
        std::cout << "Gazebo server setup failed!\n";
    }

    int dim = TOTAL_NUMBER_CONTROLLER_PARAMETERS; // problem dimensions.
    std::vector<double> x0(dim, 0.1);
    double sigma = 0.1;
    CMAParameters<> cmaparams(x0, sigma, POPULATION_SIZE);
    FitFunc dummyFunc = [](const double *x, const int N){ return 0; };
    ESOptimizer<WalkTrainer, CMAParameters<>> optim(dummyFunc, cmaparams);
    while (!optim.stop()) {
        dMat candidates = optim.ask();
        optim.eval(candidates);
        optim.tell();
        optim.inc_iter(); // important step: signals next iteration.
    }
    std::cout << optim.get_solutions() << std::endl;

    interactive_marker_server.reset(new interactive_markers::InteractiveMarkerServer("WalkTrainer"));

    gazebo::shutdown();
}