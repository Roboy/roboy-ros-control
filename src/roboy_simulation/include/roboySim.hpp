#pragma once
// common definitions
#include "CommunicationData.h"
#include "CommonDefinitions.h"
#include "common_utilities/Initialize.h"
#include "common_utilities/EmergencyStop.h"
#include "common_utilities/Record.h"
#include "common_utilities/RecordResult.h"
#include <common_utilities/Steer.h>
#include "common_utilities/Trajectory.h"
#include "common_utilities/RoboyState.h"
// ros
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/LoadController.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_parser.h>
#include <pluginlib/class_list_macros.h>
#include <control_toolbox/pid.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <urdf/model.h>
//std
#include <vector>
#include <thread>
// muscle plugin
#include "muscle/IMuscle.hpp"
// messages
#include "roboy_simulation/Tendon.h"
#include "roboy_simulation/VisualizationControl.h"
#include "roboy_simulation/ForceTorque.h"

using namespace std;
using namespace gazebo;

namespace gazebo_ros_control {

    class RoboySim : public gazebo_ros_control::RobotHWSim, public gazebo::ModelPlugin {
    public:
        /**
         * Constructor
         */
        RoboySim();

        /**
         * Destructor
         */
        ~RoboySim();

        void initializeControllers( const common_utilities::Initialize::ConstPtr& msg );

        /**
         * Overloaded Gazebo entry point
         */
        void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

        /**
         * dummy overload, our simulation is initialized in the initializeControllers callback
         */
        bool initSim(const string &robot_namespace,
                     ros::NodeHandle model_nh,
                     gazebo::physics::ModelPtr parent_model,
                     const urdf::Model *const urdf_model,
                     vector<transmission_interface::TransmissionInfo> transmissions){};

        /** Read from Simulation */
        void readSim(ros::Time time, ros::Duration period);

        /** Write to Simulation */
        void writeSim(ros::Time time, ros::Duration period);

        /** Called at each sim step */
        void Update();

        /** Called on world reset */
        void Reset();
    private:
        void visualization_control(const roboy_simulation::VisualizationControl::ConstPtr &msg);

        void publishTendon();

        void publishCOM();

        void publishForce();

        /** calculates the COM */
        void calculateCOM(int type, math::Vector3 &COM);
        /*
         * This function loads the controllers registered to the individual joint interfaces
         * @param controllers names of controllers
         * @return success
         */
        /** Calculate the Angle between links*/
        vector<double> calculateAngle_links(vector<pair<std::string, std::string>> _linkpair, int flag);
        /*
         * This function will calculate the angles between several link-pairs
         */
        map<string,math::Vector3> calculateTrunk();
        /*
         * This function will calculate the angles and velocity of the trunk
         */
        bool loadControllers(vector<string> controllers);
        /*
         * This function unloads the controllers registered to the individual joint interfaces
         * @param controllers names of controllers
         * @return success
         */
        bool unloadControllers(vector<string> controllers);
        /*
         * This function starts the controllers registered to the individual joint interfaces
         * @param controllers names of controllers
         * @return success
         */
        bool startControllers(vector<string> controllers);
        /*
         * This function stops the controllers registered to the individual joint interfaces
         * @param controllers names of controllers
         * @return success
         */
        bool stopControllers(vector<string> controllers);
        /**
         * SERVICE This function record the trajectories of the requested motors
         * @param req vector<int8> containing requested motor ids
         * @param res vector<ControllerStates> cf. CommonDefinitions.h
         */
        void record( const common_utilities::Record::ConstPtr& msg );

        /**
         * SUBSCRIBER enables pause/resume and stop recording
         */
        void steer_record(const common_utilities::Steer::ConstPtr &msg);

        void eStopCB(const std_msgs::BoolConstPtr &e_stop_active);

        bool parseMyoMuscleSDF(const string &sdf, vector<roboy_simulation::MyoMuscleInfo>& myoMuscles);

        //! ros node handle
        ros::NodeHandle *nh;

        //! Controller manager
        thread *update_thread;
        controller_manager::ControllerManager *cm = nullptr;
        double *cmd,*pos, *vel, *eff;

        int8_t recording;
        bool initialized = false;

        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        hardware_interface::EffortJointInterface jnt_eff_interface;

        ros::Subscriber steer_recording_sub, record_sub, init_sub, roboy_visualization_control_sub;
        ros::Publisher roboy_pub, recordResult_pub, visualizeTendon_pub, marker_visualization_pub;

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
        urdf::Model urdf_model;

        joint_limits_interface::EffortJointSaturationInterface ej_sat_interface;
        joint_limits_interface::EffortJointSoftLimitsInterface ej_limits_interface;
        joint_limits_interface::PositionJointSaturationInterface pj_sat_interface;
        joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface;
        joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface;
        joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface;

        vector<int> joint_types;
        vector<double> joint_lower_limits;
        vector<double> joint_upper_limits;
        vector<double> joint_effort_limits;
        vector<int> joint_control_methods;
        vector<control_toolbox::Pid> pid_controllers;

        vector<gazebo::physics::JointPtr> sim_joints;
        boost::shared_ptr<pluginlib::ClassLoader<roboy_simulation::IMuscle>> class_loader;
        vector<boost::shared_ptr<roboy_simulation::IMuscle>> sim_muscles;
        vector<roboy_simulation::MyoMuscleInfo> myoMuscles;

        enum{
            POSITION = 0,
            VELOCITY
        };

        enum{
            Tendon,
            COM,
            Force
        }visualization;

        bool visualizeTendon = false, visualizeCOM = false, visualizeForce = false;

        enum LEG_STATE{
            Stance,
            Lift_off,
            Swing,
            Stance_Preparation
        };

        LEG_STATE NextState(LEG_STATE s)
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

        LEG_STATE left_leg_state, right_leg_state;

        ros::Subscriber force_torque_ankle_left_sub, force_torque_ankle_right_sub;

        void finite_state_machine(const roboy_simulation::ForceTorque::ConstPtr &msg);

        double F_contact = 10.0, d_lift = 0.0, d_prep = 0.0; // to be optimized

        // desired user values
        double theta_heading = 0.0;
        double omega_heading = 0.0;
        double v_forward = 1.0;
    };
}
