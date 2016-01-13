#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <vector>
#include "common_utilities/Initialize.h"
#include <controller_manager_msgs/LoadController.h>
#include "FlexRayHardwareInterface.hpp"
#include "CommonDefinitions.h"

using namespace std;

class Roboy : public hardware_interface::RobotHW
{
    public:
        /**
         * Constructor
         */
	Roboy();
        /**
         * SERVICE This function initialises the requested motors
         * @param req vector<int8> containing requested motor ids
         * @param res vector<ControllerStates> cf. CommonDefinitions.h
         */
		bool initializeService(common_utilities::Initialize::Request  &req,
									  common_utilities::Initialize::Response &res);
        /**
         * Destructor
         */
        ~Roboy();
	/**
         * Read info from hardware
         */
	void read();
        /**
         * Write to Hardware
         */
	void write();
        
	bool ready = false;
    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::EffortJointInterface jnt_eff_interface;
        double *cmd;
        double *pos;
        double *vel;
        double *eff;
        ros::Time prevTime;
        FlexRayHardwareInterface flexray;
        //! ros handler
        ros::NodeHandle nh;
        ros::ServiceClient controller_manager_client;
		ros::ServiceServer init_srv;

        //publisher for nao speech
        ros::Publisher speech_pub;
};
