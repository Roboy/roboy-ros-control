#pragma once

#include <controller_manager/controller_manager.h>
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
         * Destructor
         */
        ~HardwareInterface();
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

class Roboy{
public:
	Roboy()
	{
		cm = new controller_manager::ControllerManager(&hardwareInterface);
		// this is for asyncronous ros callbacks
		ros::AsyncSpinner spinner(1);
		spinner.start();
	}

	void main_loop()
	{
		// Control loop
		ros::Time prev_time = ros::Time::now();
		ros::Rate rate(10);

		while (ros::ok())
		{
			if(hardwareInterface.ready){
				const ros::Time	    time = ros::Time::now();
				const ros::Duration period = time - prev_time;

				hardwareInterface.read();
				cm->update(time, period);
				hardwareInterface.write();

				rate.sleep();
				ROS_INFO_THROTTLE(1,"roboy ready");
			}else{
				ROS_INFO_THROTTLE(1,"roboy not ready");
			}
		}
	}

private:
	controller_manager::ControllerManager* cm;
	HardwareInterface hardwareInterface;
};

