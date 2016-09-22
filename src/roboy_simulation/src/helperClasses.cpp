#include "helperClasses.hpp"

void CoordinateSystem::Update(){
    gazebo::math::Pose pose = m_link->GetWorldPose();
    rot = pose.rot;
    origin = pose.pos;
    X = pose.rot.RotateVector(gazebo::math::Vector3::UnitX);
    Y = pose.rot.RotateVector(gazebo::math::Vector3::UnitY);
    Z = pose.rot.RotateVector(gazebo::math::Vector3::UnitZ);
}

void CoordinateSystem::UpdateHeading(){
    gazebo::math::Pose pose = m_link->GetWorldPose();
    gazebo::math::Quaternion q(0,0,pose.rot.GetAsEuler().z);
    origin = pose.pos;
    X = q.RotateVector(gazebo::math::Vector3::UnitX);
    Y = q.RotateVector(gazebo::math::Vector3::UnitY);
    Z = q.RotateVector(gazebo::math::Vector3::UnitZ);
}

void controllerParametersToMessage(vector<double> &params, roboy_simulation::ControllerParameters &msg){

    msg.F_contact = params[F_contact];
    msg.d_lift = params[d_lift];
    msg.d_prep = params[d_prep];
    msg.F_max = params[F_max];
    msg.k_v = params[k_v];
    msg.k_h = params[k_h];
    msg.k_p_theta_left.assign(params.begin()+k_p_theta_left, params.begin()+k_p_theta_left+4);
    msg.k_d_phi.assign(params.begin()+k_d_phi,params.begin()+k_d_phi+2);
    msg.k_p_theta_right.assign(params.begin()+k_p_theta_right, params.begin()+k_p_theta_right+4);
    msg.k_d_theta_left.assign(params.begin()+k_d_theta_left, params.begin()+k_d_theta_left+4);
    msg.k_d_theta_right.assign(params.begin()+k_d_theta_right, params.begin()+k_d_theta_right+4);
    msg.k_p_phi.assign(params.begin()+k_p_theta_left, params.begin()+k_p_theta_left+4);
    msg.k_d_phi.assign(params.begin()+k_p_theta_left, params.begin()+k_p_theta_left+4);
    msg.k_V = params[k_V];
    msg.k_P = params[k_P];
    msg.k_Q = params[k_Q];
    msg.k_omega = params[k_omega];
    msg.k_M_Fplus = params[k_M_Fplus];
    msg.c_hip_lift = params[c_hip_lift];
    msg.c_knee_lift = params[c_knee_lift];
    msg.c_stance_lift = params[c_stance_lift];
    msg.c_swing_prep = params[c_swing_prep];
    msg.theta_groin_0.assign(params.begin()+theta_groin_0, params.begin()+theta_groin_0+2);
    msg.phi_groin_0.assign(params.begin()+phi_groin_0, params.begin()+phi_groin_0+2);
    msg.theta_trunk_0 = params[theta_trunk_0];
    msg.phi_trunk_0 = params[phi_trunk_0];
    msg.theta_knee.assign(params.begin()+theta_knee, params.begin()+theta_knee+2);
    msg.theta_ankle.assign(params.begin()+theta_ankle, params.begin()+theta_ankle+2);
    msg.d_s.assign(params.begin()+d_s, params.begin()+d_s+2);
    msg.d_c.assign(params.begin()+d_c, params.begin()+d_c+2);
    msg.v_s.assign(params.begin()+v_s, params.begin()+v_s+2);
    msg.v_c.assign(params.begin()+v_c, params.begin()+v_c+2);
}

void messageTocontrollerParameters(const roboy_simulation::ControllerParameters::ConstPtr &msg,
                                   vector<double> &params){
    params[F_contact] = msg->F_contact;
    params[d_lift] = msg->d_lift;
    params[d_prep] = msg->d_prep;
    params[F_max] = msg->F_max;
    params[k_v] = msg->k_v;
    params[k_h] = msg->k_h;
    memcpy(&params[k_p_theta_left], msg->k_p_theta_left.data(), 4*sizeof(double));
    memcpy(&params[k_d_phi], msg->k_d_phi.data(), 2*sizeof(double));
    memcpy(&params[k_p_theta_right], msg->k_p_theta_right.data(), 4*sizeof(double));
    memcpy(&params[k_d_theta_left], msg->k_d_theta_left.data(), 4*sizeof(double));
    memcpy(&params[k_d_theta_right], msg->k_d_theta_right.data(), 4*sizeof(double));
    memcpy(&params[k_p_phi], msg->k_p_phi.data(), 4*sizeof(double));
    memcpy(&params[k_d_phi], msg->k_d_phi.data(), 4*sizeof(double));
    params[k_V] = msg->k_V;
    params[k_P] = msg->k_P;
    params[k_Q] = msg->k_Q;
    params[k_omega] = msg->k_omega;
    params[k_M_Fplus] = msg->k_M_Fplus;
    params[c_hip_lift] = msg->c_hip_lift;
    params[c_knee_lift] = msg->c_knee_lift;
    params[c_stance_lift] = msg->c_stance_lift;
    params[c_swing_prep] = msg->c_swing_prep;
    memcpy(&params[theta_groin_0], msg->theta_groin_0.data(), 2*sizeof(double));
    memcpy(&params[phi_groin_0], msg->phi_groin_0.data(), 2*sizeof(double));
    params[theta_trunk_0] = msg->theta_trunk_0;
    params[phi_trunk_0] = msg->phi_trunk_0;
    memcpy(&params[theta_knee], msg->theta_knee.data(), 2*sizeof(double));
    memcpy(&params[theta_ankle], msg->theta_ankle.data(), 2*sizeof(double));
    memcpy(&params[d_s], msg->d_s.data(), 2*sizeof(double));
    memcpy(&params[d_c], msg->d_c.data(), 2*sizeof(double));
    memcpy(&params[v_s], msg->v_s.data(), 2*sizeof(double));
    memcpy(&params[v_c], msg->v_c.data(), 2*sizeof(double));
}

void messageTocontrollerParameters(roboy_simulation::ControllerParameters &msg,
                                   vector<double> &params){
    params[F_contact] = msg.F_contact;
    params[d_lift] = msg.d_lift;
    params[d_prep] = msg.d_prep;
    params[F_max] = msg.F_max;
    params[k_v] = msg.k_v;
    params[k_h] = msg.k_h;
    memcpy(&params[k_p_theta_left], msg.k_p_theta_left.data(), 4*sizeof(double));
    memcpy(&params[k_d_phi], msg.k_d_phi.data(), 2*sizeof(double));
    memcpy(&params[k_p_theta_right], msg.k_p_theta_right.data(), 4*sizeof(double));
    memcpy(&params[k_d_theta_left], msg.k_d_theta_left.data(), 4*sizeof(double));
    memcpy(&params[k_d_theta_right], msg.k_d_theta_right.data(), 4*sizeof(double));
    memcpy(&params[k_p_phi], msg.k_p_phi.data(), 4*sizeof(double));
    memcpy(&params[k_d_phi], msg.k_d_phi.data(), 4*sizeof(double));
    params[k_V] = msg.k_V;
    params[k_P] = msg.k_P;
    params[k_Q] = msg.k_Q;
    params[k_omega] = msg.k_omega;
    params[k_M_Fplus] = msg.k_M_Fplus;
    params[c_hip_lift] = msg.c_hip_lift;
    params[c_knee_lift] = msg.c_knee_lift;
    params[c_stance_lift] = msg.c_stance_lift;
    params[c_swing_prep] = msg.c_swing_prep;
    memcpy(&params[theta_groin_0], msg.theta_groin_0.data(), 2*sizeof(double));
    memcpy(&params[phi_groin_0], msg.phi_groin_0.data(), 2*sizeof(double));
    params[theta_trunk_0] = msg.theta_trunk_0;
    params[phi_trunk_0] = msg.phi_trunk_0;
    memcpy(&params[theta_knee], msg.theta_knee.data(), 2*sizeof(double));
    memcpy(&params[theta_ankle], msg.theta_ankle.data(), 2*sizeof(double));
    memcpy(&params[d_s], msg.d_s.data(), 2*sizeof(double));
    memcpy(&params[d_c], msg.d_c.data(), 2*sizeof(double));
    memcpy(&params[v_s], msg.v_s.data(), 2*sizeof(double));
    memcpy(&params[v_c], msg.v_c.data(), 2*sizeof(double));
}