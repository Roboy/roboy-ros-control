#pragma once
#include "FlexRayHardwareInterface.hpp"
#include "CommonDefinitions.h"
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <vector>
#include "common_utilities/Initialize.h"
#include "common_utilities/EmergencyStop.h"
#include "common_utilities/Record.h"
#include <common_utilities/Steer.h>
#include "common_utilities/Waypoints.h"
#include <controller_manager_msgs/LoadController.h>

using namespace std;

class HardwareInterface : public hardware_interface::RobotHW
{
public:
	/**
	 * Constructor
	 */
	HardwareInterface();
	/**
	 * SERVICE This function initialises the requested motors
	 * @param req vector<int8> containing requested motor ids
	 * @param res vector<ControllerStates> cf. CommonDefinitions.h
	 */
	bool initializeService(common_utilities::Initialize::Request  &req,
								  common_utilities::Initialize::Response &res);
	/**
	 * SERVICE This function record the trajectories of the requested motors
	 * @param req vector<int8> containing requested motor ids
	 * @param res vector<ControllerStates> cf. CommonDefinitions.h
	 */
	bool recordService(common_utilities::Record::Request  &req,
						   common_utilities::Record::Response &res);
	/**
	 * SUBSCRIBER enables pause/resume and stop recording
	 */
	void steer_recording(const common_utilities::Steer::ConstPtr& msg);
	/**
	 * Destructor
	 */
	~HardwareInterface();
	/**
	 * Read from hardware
	 */
void read();
	/**
	 * Write to Hardware
	 */
	void write();
        
	bool ready = false;
	hardware_interface::JointStateInterface jnt_state_interface;

	hardware_interface::PositionJointInterface jnt_pos_interface;
	hardware_interface::VelocityJointInterface jnt_vel_interface;
	hardware_interface::EffortJointInterface jnt_eff_interface;

	ros::ServiceClient cm_LoadController, cm_ListController, cm_ListControllerTypes, cm_SwitchController;
private:
	double *cmd;
	double *pos;
	double *vel;
	double *eff;
	ros::Time prevTime;
	FlexRayHardwareInterface flexray;
	int8_t recording;
	//! ros handler
	ros::NodeHandle nh;
	ros::ServiceServer init_srv, record_srv;
	ros::Subscriber steer_recording_sub;
};

class Roboy : public gazebo_ros_control::RobotHWSim{
public:
	Roboy();

	void main_loop();
	bool emergencyStopService(common_utilities::EmergencyStop::Request &req,
							  common_utilities::EmergencyStop::Response &res);

	bool initSim (const std::string &robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
			 const urdf::Model *const urdf_model, std::vector< transmission_interface::TransmissionInfo > transmissions){
		// TODO: initialize simulation
	};
	void readSim (ros::Time time, ros::Duration period){
		// TODO: write data from simulation to hardwareInterface.flexray.GanglionData
	};
	void writeSim (ros::Time time, ros::Duration period){
		// TODO: write control values from hardwareInterface.flexray.controllerOutput to simulation
	};
	void eStopActive (const bool active){};

private:
	bool emergencyStop = false;
	ros::NodeHandle nh;
	controller_manager::ControllerManager* cm;
	HardwareInterface hardwareInterface;
	ros::ServiceServer emergencyStop_srv;
};

