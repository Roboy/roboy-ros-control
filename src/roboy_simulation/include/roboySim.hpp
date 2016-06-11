#pragma once
// common definitions
#include "CommonDefinitions.h"
#include "common_utilities/Initialize.h"
#include "common_utilities/EmergencyStop.h"
#include "common_utilities/Record.h"
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

using namespace std;

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
		/**
		 * SERVICE This function initialises the requested motors
		 * @param req vector<int8> containing requested motor ids
		 * @param res vector<ControllerStates> cf. CommonDefinitions.h
		 */
		bool initializeService(common_utilities::Initialize::Request &req,
							   common_utilities::Initialize::Response &res);

		/**
		 * Overloaded Gazebo entry point
		 */
		virtual void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

		/**
		 * Read from Simulation
		 */
		void readSim(ros::Time time, ros::Duration period);

		/**
		 * Write to Simulation
		 */
		void writeSim(ros::Time time, ros::Duration period);

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
		bool recordService(common_utilities::Record::Request &req,
						   common_utilities::Record::Response &res);

		/**
		 * SUBSCRIBER enables pause/resume and stop recording
		 */
		void steer_record(const common_utilities::Steer::ConstPtr &msg);

//	controller_manager::ControllerManager *cm = nullptr;
		double *cmd;
		double *pos;
		double *vel;
		double *eff;
		ros::Time prevTime;
		int8_t recording;
		bool initialized = false;

		hardware_interface::JointStateInterface jnt_state_interface;

		hardware_interface::PositionJointInterface jnt_pos_interface;
		hardware_interface::VelocityJointInterface jnt_vel_interface;
		hardware_interface::EffortJointInterface jnt_eff_interface;

		ros::ServiceServer init_srv, record_srv;
		ros::Subscriber steer_recording_sub;
		ros::Publisher roboy_pub;

		common_utilities::RoboyState roboyStateMsg;
	public:
		bool initSim(const std::string &robot_namespace,
					 ros::NodeHandle model_nh,
					 gazebo::physics::ModelPtr parent_model,
					 const urdf::Model *const urdf_model,
					 std::vector<transmission_interface::TransmissionInfo> transmissions);

		void Update();

		// Called on world reset
		void Reset();

		// Get the URDF XML from the parameter server
		std::string getURDF(std::string param_name) const;

		// Get Transmissions from the URDF
		bool parseTransmissionsFromURDF(const std::string &urdf_string);

	protected:
		void eStopCB(const std_msgs::BoolConstPtr &e_stop_active);

		// ros node handle
		ros::NodeHandle nh;

		// Pointer to the model
		gazebo::physics::ModelPtr parent_model_;
		sdf::ElementPtr sdf_;

		// deferred load in case ros is blocking
		boost::thread deferred_load_thread_;

		// Pointer to the update event connection
		gazebo::event::ConnectionPtr update_connection_;

		void load_robot_hw_sim_srv();

		// Strings
		std::string robot_namespace_;
		std::string robot_description_;

		// Transmissions in this plugin's scope
		std::vector<transmission_interface::TransmissionInfo> transmissions_;

		// Robot simulator interface
		std::string robot_hw_sim_type_str_;

		// Controller manager
		controller_manager::ControllerManager *cm = nullptr;
//		boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

		// Timing
		ros::Duration control_period_;
		ros::Time last_update_sim_time_ros_;
		ros::Time last_write_sim_time_ros_;

		// e_stop_active_ is true if the emergency stop is active.
		bool e_stop_active_, last_e_stop_active_;
		ros::Subscriber e_stop_sub_;  // Emergency stop subscriber

		// Methods used to control a joint.
		enum ControlMethod {
			EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID
		};

		// Register the limits of the joint specified by joint_name and joint_handle. The limits are
		// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
		// Return the joint's type, lower position limit, upper position limit, and effort limit.
		void registerJointLimits(const std::string &joint_name,
								 const hardware_interface::JointHandle &joint_handle,
								 const ControlMethod ctrl_method,
								 const ros::NodeHandle &joint_limit_nh,
								 const urdf::Model *const urdf_model,
								 int *const joint_type, double *const lower_limit,
								 double *const upper_limit, double *const effort_limit);

		unsigned int n_dof_;

		joint_limits_interface::EffortJointSaturationInterface ej_sat_interface_;
		joint_limits_interface::EffortJointSoftLimitsInterface ej_limits_interface_;
		joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
		joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
		joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
		joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

		std::vector<std::string> joint_names_;
		std::vector<int> joint_types_;
		std::vector<double> joint_lower_limits_;
		std::vector<double> joint_upper_limits_;
		std::vector<double> joint_effort_limits_;
		std::vector<ControlMethod> joint_control_methods_;
		std::vector<control_toolbox::Pid> pid_controllers_;

		std::vector<gazebo::physics::JointPtr> sim_joints_;
	};

}