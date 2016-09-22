#include "walkVisualization.hpp"

WalkVisualization::WalkVisualization(){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "WalkVisualization",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);
    visualization_control_sub = nh->subscribe("/roboy/visualization_control", 10,
                                              &WalkVisualization::visualization_control, this);
    leg_state_pub = nh->advertise<roboy_simulation::LegState>("/roboy/leg_state", 2);
    simulation_state_pub = nh->advertise<roboy_simulation::ControllerParameters>("/roboy/simulationState", 1);
}

void WalkVisualization::visualization_control(const roboy_simulation::VisualizationControl::ConstPtr &msg) {
    if(msg->roboyID == ID) { // only react to messages with my ID
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

void WalkVisualization::publishTendon(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles) {
//    static bool add = true;
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    char tendonnamespace[20];
    sprintf(tendonnamespace, "tendon_%d", ID);
    line_strip.ns = tendonnamespace;
//    if (add) {
    line_strip.action = visualization_msgs::Marker::ADD;
//        add = false;
//    } else {
//        line_strip.action = visualization_msgs::Marker::MODIFY;
//    }
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.003;
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;
    line_strip.pose.orientation.w = 1.0;
    line_strip.lifetime = ros::Duration(0);

    for (uint muscle = 0; muscle < sim_muscles->size(); muscle++) {
        line_strip.points.clear();
        line_strip.id = message_counter++;
        for (uint i = 0; i < (*sim_muscles)[muscle]->viaPoints.size(); i++) {
            geometry_msgs::Point p;
            p.x = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.x;
            p.y = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.y;
            p.z = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.z;
            line_strip.points.push_back(p);
            p.x = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.x;
            p.y = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.y;
            p.z = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.z;
            line_strip.points.push_back(p);
        }
        marker_visualization_pub.publish(line_strip);
    }
}

void WalkVisualization::publishCOM(math::Vector3 *center_of_mass) {
//    static bool add = true;
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    char comnamespace[20];
    sprintf(comnamespace, "COM_%d", ID);
    sphere.ns = comnamespace;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.color.r = 0.0f;
    sphere.color.g = 0.0f;
    sphere.color.b = 1.0f;
    sphere.color.a = 1.0;
    sphere.lifetime = ros::Duration(0);
    sphere.scale.x = 0.1;
    sphere.scale.y = 0.1;
    sphere.scale.z = 0.1;
//    if (add) {
    sphere.action = visualization_msgs::Marker::ADD;
//        add = false;
//    } else {
//        sphere.action = visualization_msgs::Marker::MODIFY;
//    }
    sphere.header.stamp = ros::Time::now();
    sphere.points.clear();
    sphere.id = message_counter++;
    sphere.pose.position.x = center_of_mass[POSITION].x;
    sphere.pose.position.y = center_of_mass[POSITION].y;
    sphere.pose.position.z = center_of_mass[POSITION].z;
    marker_visualization_pub.publish(sphere);
}

void WalkVisualization::publishForce(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles) {
//    static bool add = true;
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    char forcenamespace[20];
    sprintf(forcenamespace, "force_%d", ID);
    arrow.ns = forcenamespace;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration(0);
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.03;
    arrow.scale.z = 0.03;
//    if (add) {
    arrow.action = visualization_msgs::Marker::ADD;
//        add = false;
//    } else {
//        arrow.action = visualization_msgs::Marker::MODIFY;
//    }

    for (uint muscle = 0; muscle < sim_muscles->size(); muscle++) {
        for (uint i = 0; i < (*sim_muscles)[muscle]->viaPoints.size(); i++) {
            // actio
            arrow.id = message_counter++;
            arrow.color.r = 0.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.header.stamp = ros::Time::now();
            arrow.points.clear();
            geometry_msgs::Point p;
            p.x = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.x;
            p.y = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.y;
            p.z = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.z;
            arrow.points.push_back(p);
            p.x += (*sim_muscles)[muscle]->viaPoints[i]->prevForce.x;
            p.y += (*sim_muscles)[muscle]->viaPoints[i]->prevForce.y;
            p.z += (*sim_muscles)[muscle]->viaPoints[i]->prevForce.z;
            arrow.points.push_back(p);
            marker_visualization_pub.publish(arrow);
            // reactio
            arrow.id = message_counter++;
            arrow.color.r = 1.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.header.stamp = ros::Time::now();
            arrow.points.clear();
            p.x = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.x;
            p.y = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.y;
            p.z = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.z;
            arrow.points.push_back(p);
            p.x += (*sim_muscles)[muscle]->viaPoints[i]->nextForce.x;
            p.y += (*sim_muscles)[muscle]->viaPoints[i]->nextForce.y;
            p.z += (*sim_muscles)[muscle]->viaPoints[i]->nextForce.z;
            arrow.points.push_back(p);
            marker_visualization_pub.publish(arrow);
        }
    }
}

void WalkVisualization::publishMomentArm(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles) {
//    static bool add = true;
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    char momentarmnamespace[20];
    sprintf(momentarmnamespace, "momentarm_%d", ID);
    arrow.ns = momentarmnamespace;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.r = 0.0f;
    arrow.color.g = 0.0f;
    arrow.color.b = 1.0f;
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration(0);
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.03;
    arrow.scale.z = 0.03;
//    if (add) {
    arrow.action = visualization_msgs::Marker::ADD;
//        add = false;
//    } else {
//        arrow.action = visualization_msgs::Marker::MODIFY;
//    }
    for (uint muscle = 0; muscle < sim_muscles->size(); muscle++) {
        arrow.id = message_counter++;
        arrow.header.stamp = ros::Time::now();
        arrow.points.clear();
        geometry_msgs::Point p;
        math::Pose jointPose = (*sim_muscles)[muscle]->spanningJoint->GetWorldPose();
        p.x = jointPose.pos.x;
        p.y = jointPose.pos.y;
        p.z = jointPose.pos.z;
        arrow.points.push_back(p);
        p.x += (*sim_muscles)[muscle]->momentArm.x;
        p.y += (*sim_muscles)[muscle]->momentArm.y;
        p.z += (*sim_muscles)[muscle]->momentArm.z;
        arrow.points.push_back(p);
        marker_visualization_pub.publish(arrow);
    }
}

void WalkVisualization::publishModel(vector<string> &link_names, physics::ModelPtr parent_model){
//    static bool add = true;
    visualization_msgs::Marker mesh;
    mesh.header.frame_id = "world";
    char modelnamespace[20];
    sprintf(modelnamespace, "model_%d", ID);
    mesh.ns = modelnamespace;
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.color.r = 1.0f;
    mesh.color.g = 1.0f;
    mesh.color.b = 1.0f;
    mesh.color.a = 0.5;
    mesh.scale.x = 1.0;
    mesh.scale.y = 1.0;
    mesh.scale.z = 1.0;
    mesh.lifetime = ros::Duration(0);
    mesh.header.stamp = ros::Time::now();
//    if (add) {
    mesh.action = visualization_msgs::Marker::ADD;
//        add = false;
//    } else {
//        mesh.action = visualization_msgs::Marker::MODIFY;
//    }
    for(auto link_name:link_names){
        mesh.id = message_counter++;
        physics::LinkPtr link = parent_model->GetLink(link_name);
        math::Pose pose = link->GetWorldPose();
        pose.rot.Normalize();
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

void WalkVisualization::publishSimulationState(ControllerParameters &params, gazebo::common::Time gz_time_now){
    roboy_simulation::ControllerParameters msg;
    msg.roboyID = ID;
    controllerParametersToMessage(params, msg);
    msg.sim_time = gz_time_now.Float();
    simulation_state_pub.publish(msg);
}


void WalkVisualization::publishLegState(LEG_STATE *leg_state){
    roboy_simulation::LegState msgLeft;
    msgLeft.roboyID = ID;
    msgLeft.leg = LEG::LEFT;
    msgLeft.state = leg_state[LEG::LEFT];
    leg_state_pub.publish(msgLeft);
    roboy_simulation::LegState msgRight;
    msgRight.roboyID = ID;
    msgRight.leg = LEG::RIGHT;
    msgRight.state = leg_state[LEG::RIGHT];
    leg_state_pub.publish(msgRight);
}

void WalkVisualization::publishStateMachineParameters(math::Vector3 *center_of_mass,
                                                      math::Vector3 *foot_sole_global,
                                                      CoordSys hip_CS, ControllerParameters &params){
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    char momentarmnamespace[20];
    sprintf(momentarmnamespace, "statemachineparams_%d", ID);
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
    p.x += params[d_s+LEG::LEFT]*hip_CS->X.x;
    p.y += params[d_s+LEG::LEFT]*hip_CS->X.y;
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
    p.x += params[d_c+LEG::LEFT]*hip_CS->Y.x;
    p.y += params[d_c+LEG::LEFT]*hip_CS->Y.y;
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
    p.x += params[d_s+LEG::RIGHT]*hip_CS->X.x;
    p.y += params[d_s+LEG::RIGHT]*hip_CS->X.y;
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
    p.x += params[d_c+LEG::RIGHT]*hip_CS->Y.x;
    p.y += params[d_c+LEG::RIGHT]*hip_CS->Y.y;
    p.z = 0;
    arrow.points.push_back(p);
    marker_visualization_pub.publish(arrow);
    text.id = message_counter++;
    text.pose.position = p;
    text.text = "d_c[right]";
    marker_visualization_pub.publish(text);
}

void WalkVisualization::publishCoordinateSystems(physics::LinkPtr parent_link, ros::Time time, bool child_link){
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