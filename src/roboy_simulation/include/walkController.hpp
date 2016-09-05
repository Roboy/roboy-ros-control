#pragma once
// std
#include <cstdlib>
#include <iostream>
#include <deque>
// ros
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <visualization_msgs/Marker.h>
#include <transmission_interface/transmission_parser.h>
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
// boost
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
// muscle plugin
#include "DummyMusclePlugin.hpp"
// ros messages
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include "roboy_simulation/Tendon.h"
#include "roboy_simulation/VisualizationControl.h"
#include "roboy_simulation/ForceTorque.h"
#include "roboy_simulation/LegState.h"
#include "common_utilities/Initialize.h"
#include "common_utilities/EmergencyStop.h"
#include "common_utilities/Record.h"
#include "common_utilities/RecordResult.h"
#include <common_utilities/Steer.h>
#include "common_utilities/Trajectory.h"
#include "common_utilities/RoboyState.h"

using namespace gazebo;
using namespace std;

static const char * FOOT[] = { "foot_left", "foot_right" };

static const char * LEG_STATE_STRING[] = { "Stance", "Lift_off", "Swing", "Stance_Preparation" };

static const char * LEG_NAMES_STRING[] = { "left leg", "right leg" };

enum{
    POSITION = 0,
    VELOCITY
};

enum PLANE{
    TRAVERSAL,
    SAGITTAL,
    CORONAL
};

class WalkController : public gazebo::ModelPlugin{
public:
    WalkController();
    ~WalkController();

    /**
     * Overloaded Gazebo entry point
     */
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    /** Called at each sim step */
    void Update();

    /** Read from Simulation */
    void readSim(ros::Time time, ros::Duration period);

    /** Write to Simulation */
    void writeSim(ros::Time time, ros::Duration period);

    /** Called on world reset */
    void Reset();

    void finite_state_machine(const roboy_simulation::ForceTorque::ConstPtr &msg);

    LEG_STATE NextState(LEG_STATE s);

    LEG getLegInState(LEG_STATE s);

    bool updateFootDisplacementAndVelocity();

    bool updateTargetFeatures();

    void updateMuscleForces();

    void updateMuscleActivities();

    void visualization_control(const roboy_simulation::VisualizationControl::ConstPtr &msg);

    LEG_STATE leg_state[2];

    bool visualizeTendon = false, visualizeCOM = false, visualizeForce = false, visualizeMomentArm = false;
private:
    /** Emergency stop callback */
    void eStopCB(const std_msgs::BoolConstPtr &e_stop_active);

    /** This function parses a sdf dtring for myoMuscle parameters
     * @param sdf string
     * @param myoMuscles will be filled with the paramaters
     * @return success
     */
    bool parseMyoMuscleSDF(const string &sdf, vector<roboy_simulation::MyoMuscleInfo>& myoMuscles);

    /** calculates the COM */
    void calculateCOM(int type, math::Vector3 &COM);

    /**
     * This function calculates the angles between several link-pairs
     * @param linkpair vector of link pair names
     * @param flag #PLANE (SAGITTAL, TRAVERSAL, CORONAL)
     * @return a vector containing the angles in the respective plane
     */
    vector<double> calculateAngles(vector<pair<std::string, std::string>> linkpair, PLANE flag);
/**
     * This function calculates the angle two links
     * @param link0 first link
     * @param link1 second link
     * @param flag #PLANE (SAGITTAL, TRAVERSAL, CORONAL)
     * @return angle between links in respective plane
     */
    double calculateAngle(string link0, string link1, PLANE flag);
    /**
     * This function will calculate the angles and velocity of the trunk
     */
    map<string,math::Vector3> calculateTrunk();

    void publishTendon();

    void publishCOM();

    void publishForce();

    void publishMomentArm();

    ros::NodeHandle *nh;
    ros::Subscriber force_torque_ankle_left_sub, force_torque_ankle_right_sub, roboy_visualization_control_sub;
    ros::Publisher visualizeTendon_pub, marker_visualization_pub;
    vector<string> link_names;

    double gazebo_max_step_size = 0.003;

    double F_contact = 10.0, d_lift = -0.3, d_prep = 0.0;
    double F_max = 500;
    // desired user values
    double psi_heading = 0.0;
    double omega_heading = 0.0;
    double v_forward = 1.0;
    double v_COM;

    // target feature gains
    double k_v, k_h, k_p_theta_left[4], k_p_theta_right[4], k_d_theta_left[4], k_d_theta_right[4], k_p_phi[2],
            k_d_phi[2];
    // target force torque gains
    double k_V, k_P, k_Q, k_omega;
    // feedback gains
    double k_M_Fplus = 1.0, c_hip_lift = 1.0, c_lift_kee = 1.0, c_stance_lift = 1.0, c_swing_prep = 1.0;
    // target features
    map<string,math::Quaternion> Q;
    map<string,math::Vector3> P;
    map<string,math::Vector3> v;
    map<string,math::Vector3> omega;
    // target force torque
    map<string,math::Vector3> F;
    map<string,math::Vector3> T;
    map<string,double> tau;
    map<string,double> F_tilde;
    map<string,deque<double>> activity;
    map<string,double> feedback;
    map<string,double> a;

    map<string,vector<uint>> muscles_spanning_joint;

    double theta_groin_0[2], phi_groin_0[2], theta_trunk_0, phi_trunk_0, theta_knee[2], theta_ankle[2];
    double d_s, d_c, v_s, v_c;

    math::Vector3 center_of_mass[2];

    enum{
        Tendon,
        COM,
        Force,
        MomentArm
    }visualization;

    double *cmd, *pos, *vel, *eff;

    int8_t recording;

    ros::Subscriber steer_recording_sub, record_sub, init_sub, init_walk_controller_sub;
    ros::Publisher roboy_pub, recordResult_pub, leg_state_pub;

    common_utilities::RoboyState roboyStateMsg;

    // Pointer to the model
    gazebo::physics::ModelPtr parent_model;
    sdf::ElementPtr sdf;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection;

    // Strings
    string robot_namespace, robot_description;

    // Transmissions in this plugin's scope
    vector<transmission_interface::TransmissionInfo> transmissions;

    // Timing
    ros::Duration control_period;
    ros::Time last_update_sim_time_ros;
    ros::Time last_write_sim_time_ros;

    // e_stop_active_ is true if the emergency stop is active.
    bool e_stop_active, last_e_stop_active;
    ros::Subscriber e_stop_sub;  // Emergency stop subscriber

    uint numberOfMyoMuscles;

    vector<gazebo::physics::JointPtr> sim_joints;
    boost::shared_ptr<pluginlib::ClassLoader<roboy_simulation::DummyMusclePlugin>> class_loader;
    vector<boost::shared_ptr<roboy_simulation::DummyMusclePlugin>> sim_muscles;
    vector<roboy_simulation::MyoMuscleInfo> myoMuscles;
};