#pragma once

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <vector>
#include "common_utilities/Initialize.h"
#include "common_utilities/EmergencyStop.h"
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
	hardware_interface::PositionJointInterface jnt_pos_interface;
private:
	hardware_interface::JointStateInterface jnt_state_interface;
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
	NCursesInterface *interface;
};

class Roboy{
public:
	Roboy()
	{
		cm = new controller_manager::ControllerManager(&hardwareInterface);
		emergencyStop_srv = nh.advertiseService("/roboy/emergencyStop", &Roboy::emergencyStopService, this);
		// this is for asyncronous ros callbacks
	}

	void main_loop()
	{
		// Control loop
		ros::Time prev_time = ros::Time::now();
		ros::Rate rate(10);

		ros::AsyncSpinner spinner(4); // 4 threads
		spinner.start();

		bool controller_loaded = false;

		while (ros::ok() && !emergencyStop)
		{
			if(hardwareInterface.ready){
				if(controller_loaded) {
					const ros::Time time = ros::Time::now();
					const ros::Duration period = time - prev_time;

					hardwareInterface.read();
					cm->update(time, period);
					hardwareInterface.write();

					rate.sleep();
				}else{
					ROS_DEBUG_THROTTLE(1, "loading controller");
					vector<string> resources = hardwareInterface.jnt_pos_interface.getNames();
					for(auto resource : resources) {
						cm->loadController(resource);
					}
					controller_loaded = true;
					ROS_DEBUG_THROTTLE(1, "roboy ready");
				}
			}else{
				ROS_DEBUG_THROTTLE(1,"roboy not ready");
			}
		}
	}

	bool emergencyStopService(common_utilities::EmergencyStop::Request &req,
							  common_utilities::EmergencyStop::Response &res){
		if(req.all){
			emergencyStop=true;
		}else{
			for(auto i:req.idList) {
				char controllername[50];
				snprintf(controllername,50,"motor%d",i);
				cm->unloadController(controllername);
			}
		}
	}
private:
	bool emergencyStop = false;
	ros::NodeHandle nh;
	controller_manager::ControllerManager* cm;
	HardwareInterface hardwareInterface;
	ros::ServiceServer emergencyStop_srv;
};

