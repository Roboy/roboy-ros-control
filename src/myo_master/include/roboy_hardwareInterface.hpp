#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include "myo_master/InitializeRequest.h"
#include "FlexRayHardwareInterface.hpp"

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
        void initCB(myo_master::InitializeRequest msg);
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
        // subscriber to head tactile states
        ros::Subscriber Init;
        
        //publisher for nao speech
        ros::Publisher speech_pub;
};
