#pragma once

// ros
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// muscle plugin
#include "MusclePlugin.hpp"
// messages
#include "roboy_simulation/Tendon.h"
#include "roboy_simulation/VisualizationControl.h"
#include "roboy_simulation/ForceTorque.h"

using namespace gazebo;
using namespace std;

static const char * FOOT[] = { "foot_left", "foot_right" };

enum LEG_STATE{
    Stance,
    Lift_off,
    Swing,
    Stance_Preparation
};

enum{
    POSITION = 0,
    VELOCITY
};

class WalkController{
public:
    WalkController(vector<boost::shared_ptr<roboy_simulation::MusclePlugin>> &sim_muscles,
                   gazebo::physics::ModelPtr parent_model);
    ~WalkController();

    /** calculates the COM */
    void calculateCOM(int type, math::Vector3 &COM);

    /**
     * This function will calculate the angles between several link-pairs
     */
    vector<double> calculateAngle_links(vector<pair<std::string, std::string>> _linkpair, int flag);
    /**
     * This function will calculate the angles and velocity of the trunk
     */
    map<string,math::Vector3> calculateTrunk();

    void finite_state_machine(const roboy_simulation::ForceTorque::ConstPtr &msg);

    LEG_STATE NextState(LEG_STATE s);

    LEG getLegInState(LEG_STATE s);

    void calculateTargetFeatures();

    void visualization_control(const roboy_simulation::VisualizationControl::ConstPtr &msg);

    void publishTendon();

    void publishCOM();

    void publishForce();

    void publishMomentArm();

    LEG_STATE leg_state[2];
private:
    ros::NodeHandle *nh;
    ros::Subscriber force_torque_ankle_left_sub, force_torque_ankle_right_sub, roboy_visualization_control_sub;
    ros::Publisher visualizeTendon_pub, marker_visualization_pub;
    gazebo::physics::ModelPtr parent_model;
    vector<string> link_names;

    vector<boost::shared_ptr<roboy_simulation::MusclePlugin>> &sim_muscles;
    double F_contact = 10.0, d_lift = 0.0, d_prep = 0.0; // to be optimized
    // desired user values
    double psi_heading = 0.0;
    double omega_heading = 0.0;
    double v_forward = 1.0;
    double v_COM;

    // target feature gains
    double k_v, k_h, k_p_theta, k_d_theta, k_p_phi, k_d_phi;
    // target features
    map<string,math::Quaternion> Q;
    map<string,math::Vector3> P;
    map<string,math::Vector3> v;
    map<string,math::Vector3> omega;

    double theta_hip_0, phi_hip_0, theta_trunk_0, phi_trunk_0;
    double d_s, d_c, v_s, v_c;

    math::Vector3 center_of_mass[2];

    enum{
        Tendon,
        COM,
        Force,
        MomentArm
    }visualization;

    bool visualizeTendon = false, visualizeCOM = false, visualizeForce = false, visualizeMomentArm = false;
};