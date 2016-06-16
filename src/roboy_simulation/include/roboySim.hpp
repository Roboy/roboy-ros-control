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
#include <mutex>
// muscle plugin
#include "MusclePlugin.hh"

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
		 * Read from Simulation
		 */
		void readSim(ros::Time time, ros::Duration period);

		/**
		 * Write to Simulation
		 */
		void writeSim(ros::Time time, ros::Duration period);

		bool initSim(const string &robot_namespace,
					 ros::NodeHandle model_nh,
					 gazebo::physics::ModelPtr parent_model,
					 const urdf::Model *const urdf_model,
					 vector<transmission_interface::TransmissionInfo> transmissions);

		void Update();

		// Called on world reset
		void Reset();

	private:
		/*
		 * This function loads the controllers registered to the individual joint interfaces
		 * @param controlmode Position, Velocity or Force
		 * @return success
		 */
		bool loadControllers(int controlmode);
		/*
		 * This function unloads the controllers registered to the individual joint interfaces
		 * @param controlmode Position, Velocity or Force
		 * @return success
		 */
		bool unloadControllers(int controlmode);
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

		// Get the URDF XML from the parameter server
		string getURDF(string param_name) const;

		void eStopCB(const std_msgs::BoolConstPtr &e_stop_active);

		// Register the limits of the joint specified by joint_name and joint_handle. The limits are
		// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
		// Return the joint's type, lower position limit, upper position limit, and effort limit.
		void registerJointLimits(const string &joint_name,
								 const hardware_interface::JointHandle &joint_handle,
								 const int ctrl_method,
								 const ros::NodeHandle &joint_limit_nh,
								 const urdf::Model *const urdf_model,
								 int *const joint_type, double *const lower_limit,
								 double *const upper_limit, double *const effort_limit);

		bool parseMyoMuscleSDF(const string &sdf, vector<roboy_simulation::MyoMuscleInfo>& myoMuscles);

		//! ros node handle
		ros::NodeHandle nh;

		//! Controller manager
		controller_manager::ControllerManager *cm = nullptr;
		double *cmd,*pos, *vel, *eff;

		int8_t recording;
		bool initialized = false;

		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;
		hardware_interface::VelocityJointInterface jnt_vel_interface;
		hardware_interface::EffortJointInterface jnt_eff_interface;

		ros::Subscriber steer_recording_sub, record_sub, init_sub;
		ros::Publisher roboy_pub, recordResult_pub;

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

		joint_limits_interface::EffortJointSaturationInterface ej_sat_interface;
		joint_limits_interface::EffortJointSoftLimitsInterface ej_limits_interface;
		joint_limits_interface::PositionJointSaturationInterface pj_sat_interface;
		joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface;
		joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface;
		joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface;

		vector<string> joint_names;
		vector<int> joint_types;
		vector<double> joint_lower_limits;
		vector<double> joint_upper_limits;
		vector<double> joint_effort_limits;
		vector<int> joint_control_methods;
		vector<control_toolbox::Pid> pid_controllers;

		vector<gazebo::physics::JointPtr> sim_joints;
		boost::shared_ptr<pluginlib::ClassLoader<roboy_simulation::MusclePlugin>> class_loader;
		vector<boost::shared_ptr<roboy_simulation::MusclePlugin>> sim_muscles;
		vector<roboy_simulation::MyoMuscleInfo> myoMuscles;
		vector<math::Vector3> viaPointInGobalFrame, force;
	};
}