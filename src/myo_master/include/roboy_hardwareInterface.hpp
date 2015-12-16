#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <vector>
#include "myo_master/InitializeRequest.h"
#include "myo_master/InitializeResponse.h"
#include <controller_manager_msgs/LoadController.h>
#include "FlexRayHardwareInterface.hpp"

using namespace std;

class Roboy : public hardware_interface::RobotHW
{
    public:
        /**
         * Constructor
         */
	Roboy();
        /**
         * CALLBACK This function initialises the requested motors
         */
        void initializeCallback(myo_master::InitializeRequest msg);
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
        ros::Subscriber init_request_sub;
        ros::Publisher init_response_pub;
        ros::ServiceClient controller_manager_client;
        //publisher for nao speech
        ros::Publisher speech_pub;
};
