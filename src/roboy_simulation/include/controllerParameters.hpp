#pragma once

#include <vector>
using namespace std;

#define TOTAL_NUMBER_CONTROLLER_PARAMETERS 53

enum PARAMETERS{
    F_contact = 0,
    d_lift,
    d_prep,
    F_max,
    // target feature gains
    k_v,
    k_h,
    k_p_theta_left = 6,
    k_p_theta_right = 10,
    k_d_theta_left = 14,
    k_d_theta_right = 18,
    k_p_phi = 22,
    k_d_phi = 24,
    // target force torque gains
    k_V = 26,
    k_P,
    k_Q,
    k_omega,
    // feedback gains/ constant excitation
    k_M_Fplus,
    c_hip_lift,
    c_knee_lift,
    c_stance_lift,
    c_swing_prep,
    // initial pose
    theta_groin_0 = 35,
    phi_groin_0= 37,
    theta_trunk_0 = 39,
    phi_trunk_0,
    // knee and ankle pitch
    theta_knee = 41,
    theta_ankle = 43,
    // foot displacement parameters
    d_s = 45,
    d_c = 47,
    v_s = 49,
    v_c = 51
};

typedef vector<double> ControllerParameters;
