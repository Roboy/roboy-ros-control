#pragma once

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <vector>
#include "common_utilities/Initialize.h"
#include "common_utilities/EmergencyStop.h"
#include "common_utilities/Record.h"
#include <controller_manager_msgs/LoadController.h>
#include "FlexRayHardwareInterface.hpp"
#include "CommonDefinitions.h"
#include "ncurses_hardwareInterface.hpp"

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

    NCursesInterface interface;
	ros::ServiceClient cm_LoadController, cm_ListController, cm_ListControllerTypes, cm_SwitchController;
private:
	double *cmd;
	double *pos;
	double *vel;
	double *eff;
	ros::Time prevTime;
	FlexRayHardwareInterface flexray;
	//! ros handler
	ros::NodeHandle nh;
	ros::ServiceServer init_srv, record_srv;
};

class Roboy{
public:
	Roboy();

	void main_loop();

	bool emergencyStopService(common_utilities::EmergencyStop::Request &req,
							  common_utilities::EmergencyStop::Response &res);
private:
	bool emergencyStop = false;
	ros::NodeHandle nh;
	controller_manager::ControllerManager* cm;
	HardwareInterface hardwareInterface;
	ros::ServiceServer emergencyStop_srv;
};

