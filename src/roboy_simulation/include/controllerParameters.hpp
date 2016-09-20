#pragma once

struct ControllerParameters{
    double F_contact = 10.0, d_lift = -0.3, d_prep = 0.0;
    double F_max = 500;
    // desired user values
    double psi_heading = 0.0;
    double omega_heading = 0.0;
    double v_forward = 1.0;
    double v_COM;

    // target feature gains
    double k_v, k_h, k_p_theta_left[4], k_p_theta_right[4], k_d_theta_left[4],
            k_d_theta_right[4], k_p_phi[2], k_d_phi[2];
    // target force torque gains
    double k_V = 1.0, k_P = 1.0, k_Q = 1.0, k_omega = 1.0;
    // feedback gains
    double k_M_Fplus = 1.0, c_hip_lift = 1.0, c_knee_lift = 1.0,
            c_stance_lift = 0.2, c_swing_prep = 0.2;
    double theta_groin_0[2], phi_groin_0[2], theta_trunk_0,
            phi_trunk_0, theta_knee[2], theta_ankle[2];
    double d_s[2], d_c[2], v_s[2], v_c[2];
};
