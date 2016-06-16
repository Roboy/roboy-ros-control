#pragma once
#include "FlexRayHardwareInterface.hpp"
#include "CommonDefinitions.h"
// common definitions
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
#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/LoadController.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
// std
#include <vector>
#include <mutex>

using namespace std;

//! enum for state machine
typedef enum
{
	WaitForInitialize,
	LoadControllers,
	Controlloop,
	PublishState,
	Recording
} ActionState;

class Roboy : public hardware_interface::RobotHW{
public:
	/**
	 * Constructor
	 */
	Roboy();
	/**
	 * Destructor
	 */
	~Roboy();
	/**
	 * CB This function initialises the requested motors
	 */
	void initializeControllers( const common_utilities::Initialize::ConstPtr& msg );

    /**
	 * Read from hardware
	 */
	void read();
	/**
	 * Write to Hardware
	 */
	void write();
	/**
	 * This is the main loop
	 */
	void main_loop(controller_manager::ControllerManager *cm);
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
    bool startControllers(int controlmode);
    bool stopControllers(int controlmode);
	/**
	 * SERVICE This function record the trajectories of the requested motors
	 * @param req vector<int8> containing requested motor ids
	 * @param res vector<ControllerStates> cf. CommonDefinitions.h
	 */
	void record( const common_utilities::Record::ConstPtr& msg );
	/**
	 * SUBSCRIBER enables pause/resume and stop recording
	 */
	void steer_record(const common_utilities::Steer::ConstPtr& msg);

	ros::NodeHandle nh;
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

    controller_manager::ControllerManager *cm = nullptr;
	ros::ServiceClient cm_LoadController, cm_UnloadController, cm_ListController, cm_ListControllerTypes, cm_SwitchController;
	ros::Subscriber steer_recording_sub, init_sub, record_sub;
	ros::Publisher roboy_pub, recordResult_pub;

	FlexRayHardwareInterface flexray;
	common_utilities::RoboyState roboyStateMsg;

	//! current state of roboy
	ActionState currentState;
	/**
	 * Statemachine function for next state
	 * @param s current State
	 * @return next state
	 */
	ActionState NextState(ActionState s);
	//! state strings describing each state
	std::map<ActionState, std::string> state_strings = {
			{ WaitForInitialize,     "Waiting for initialization of controllers" },
			{ LoadControllers,       "Loading controllers" },
			{ Controlloop,           "Control loop" },
			{ PublishState,           "Publish roboy state" },
			{ Recording,             "Recording" }
	};
};

