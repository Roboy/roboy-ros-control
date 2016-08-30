#include "walkController.hpp"

WalkController::WalkController(vector <boost::shared_ptr<roboy_simulation::MusclePlugin>> &sim_muscles,
                               gazebo::physics::ModelPtr parent_model):
sim_muscles(sim_muscles), parent_model(parent_model){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "walkController",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = new ros::NodeHandle;

    force_torque_ankle_left_sub  = nh->subscribe("/roboy/force_torque_ankle_left", 1,
                                                 &WalkController::finite_state_machine, this);
    force_torque_ankle_right_sub  = nh->subscribe("/roboy/force_torque_ankle_right", 1,
                                                  &WalkController::finite_state_machine, this);

    roboy_visualization_control_sub = nh->subscribe("/roboy/visualization_control", 10, &WalkController::visualization_control, this);

    visualizeTendon_pub = nh->advertise<roboy_simulation::Tendon>("/visual/tendon", 1);
    marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // the following links are part of my robot (this is useful if the model.sdf contains additional links)
    link_names.push_back("hip");
    link_names.push_back("thigh_left");
    link_names.push_back("thigh_right");
    link_names.push_back("shank_left");
    link_names.push_back("shank_right");
    link_names.push_back("ankle_left");
    link_names.push_back("ankle_right");
}

WalkController::~WalkController(){
    delete nh;
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

vector<double> WalkController::calculateAngle_links(vector<pair<std::string, std::string>> _linkpair, int flag){
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

map<string,math::Vector3> WalkController::calculateTrunk(){
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

void WalkController::finite_state_machine(const roboy_simulation::ForceTorque::ConstPtr &msg){
    // check what state the leg is currently in
    bool state_transition = false;

    switch(leg_state[msg->leg]){
        case Stance:{
            if(d_s < d_lift || (leg_state[LEG::LEFT] == Stance && leg_state[LEG::RIGHT] == Stance)){
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
        leg_state[msg->leg] = NextState(leg_state[msg->leg]);
    }
}

LEG_STATE WalkController::NextState(LEG_STATE s)
{
    LEG_STATE newstate;
    switch (s)
    {
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

LEG WalkController::getLegInState(LEG_STATE s){
    if(leg_state[LEG::LEFT]==s)
        return LEG::LEFT;
    else if(leg_state[LEG::RIGHT]==s)
        return LEG::RIGHT;
    else
        return LEG::NONE;
}

void WalkController::calculateTargetFeatures(){
    // calculate the COM position and velocity
    calculateCOM(POSITION, center_of_mass[POSITION]);
    calculateCOM(VELOCITY, center_of_mass[VELOCITY]);

    // calculate signed horizontal distance between foot_pos and COM
    math::Pose hip_pose = parent_model->GetLink("hip")->GetWorldPose();

    // get the stance leg
    LEG stance_leg = getLegInState(Stance);

    // get the foot position and velocity
    math::Vector3 foot_pos = parent_model->GetLink(FOOT[stance_leg])->GetWorldPose().pos;
    math::Vector3 foot_vel = parent_model->GetLink(FOOT[stance_leg])->GetWorldCoGLinearVel();

    // calculate the distance and velocity between foot and center of mass in world frame
    math::Vector3 d_foot_pos  = foot_pos  - center_of_mass[POSITION];
    math::Vector3 d_foot_vel  = foot_vel  - center_of_mass[VELOCITY];

    // rotate into hip frame
    d_foot_pos = hip_pose.rot.RotateVector(d_foot_pos);
    d_foot_vel = hip_pose.rot.RotateVector(d_foot_vel);
    v_COM = hip_pose.rot.RotateVector(hip_pose.pos).x;

    // calculate signed sagittal and coronal foot displacement and velocity
    d_s = d_foot_pos.x;
    d_c = d_foot_pos.y;
    v_s = d_foot_vel.x;
    v_c = d_foot_vel.y;

    // target velocity
    math::Quaternion heading(0,0,psi_heading);
    math::Vector3 v_target(v_forward,0,0);
    v_target = heading.RotateVector(v_target);

    // trunk
    double theta_trunk = theta_trunk_0 + k_v * ( v_forward - v_COM );
    double phi_trunk = phi_trunk_0 + k_h * ( psi_heading - hip_pose.rot.GetYaw() );
    // target orientation for the hip
    Q["hip"] = math::Quaternion(phi_trunk, theta_trunk, psi_heading);
    // there is no target position for the hip, so we set it to the current position
    P["hip"] = hip_pose.pos;
    // target velocity is the forward velocity into desired heading direction
    v["hip"] = v_target;
    // target angular velocity is zero
    omega["hip"] = math::Vector3::Zero;

    // hip
    double theta_hip = theta_hip_0 + k_p_theta * d_s + k_d_theta * (v_forward - v_s);
    double phi_hip = phi_hip_0 + k_p_phi * d_c - k_d_phi * v_c;

}

void WalkController::visualization_control(const roboy_simulation::VisualizationControl::ConstPtr &msg){
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
        case MomentArm:{
            visualizeMomentArm = msg->value;
            break;
        }
    }
}

void WalkController::publishTendon(){
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

void WalkController::publishCOM(){
    static bool add = true;
    visualization_msgs::Marker sphere;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    sphere.header.frame_id = "hip";
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

void WalkController::publishForce(){
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
        for (uint i = 0; i < sim_muscles[muscle]->viaPointsInGlobalFrame.size(); i++) {
            arrow.id = id;
            if (fabs(sim_muscles[muscle]->force[i].GetSquaredLength()) > 0.0) {
                arrow.header.stamp = ros::Time::now();
                arrow.points.clear();
                geometry_msgs::Point p;
                p.x = sim_muscles[muscle]->viaPointsInGlobalFrame[i].x;
                p.y = sim_muscles[muscle]->viaPointsInGlobalFrame[i].y;
                p.z = sim_muscles[muscle]->viaPointsInGlobalFrame[i].z;
                arrow.points.push_back(p);
                p.x += sim_muscles[muscle]->force[i].x;
                p.y += sim_muscles[muscle]->force[i].y;
                p.z += sim_muscles[muscle]->force[i].z;
                arrow.points.push_back(p);
            } else {
                arrow.action = visualization_msgs::Marker::DELETE;
                add = true;
            }
            marker_visualization_pub.publish(arrow);
        }
    }
}

void WalkController::publishMomentArm(){
    static bool add = true;
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
    if(add) {
        arrow.action = visualization_msgs::Marker::ADD;
        add = false;
    }else{
        arrow.action = visualization_msgs::Marker::MODIFY;
    }
    uint id = 0;
    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        for (uint i = 0; i < sim_muscles[muscle]->momentArm.size(); i++) {
            arrow.id = id;
            arrow.header.stamp = ros::Time::now();
            arrow.points.clear();
            geometry_msgs::Point p;
            math::Pose linkPose = sim_muscles[muscle]->links[i]->GetWorldPose();
            p.x = linkPose.pos.x;
            p.y = linkPose.pos.y;
            p.z = linkPose.pos.z;
            arrow.points.push_back(p);
            p.x += sim_muscles[muscle]->momentArm[i].x;
            p.y += sim_muscles[muscle]->momentArm[i].y;
            p.z += sim_muscles[muscle]->momentArm[i].z;
            arrow.points.push_back(p);
            marker_visualization_pub.publish(arrow);
        }
    }
}