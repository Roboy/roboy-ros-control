#pragma once
// common definitions
#include "CommonDefinitions.h"
#include "CommunicationData.h"

#include "ITendon.hpp"
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Pose.hh>
// ros
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <CommunicationData.h>
// boost
#include <boost/numeric/odeint.hpp>
#include <boost/bind.hpp>
//std
#include <math.h>
#include <map>
#include <stdio.h>
#include <sstream>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>

namespace roboy_simulation {

	using namespace std;
	using namespace boost::numeric::odeint;
	using namespace gazebo;

	const int linkNumber = 3; //later read this number from some table or .txt files





	struct MyoMuscleInfo{
		string name;
		map<string, vector<IViaPoints>> viaPoints;
		Motor motor;
		Gear gear;
		Spindle spindle;
		SEE see;
		MyoMuscleInfo():viaPoints(){};
	};

	struct PIDcontroller{
		double calc_output(double cmd, double pos, double timePeriod);
		sint32 outputPosMax = 24; /*!< maximum control output in the positive direction in counts, max 4000*/
		sint32 outputNegMax = -24; /*!< maximum control output in the negative direction in counts, max -4000*/
		float32 spPosMax = 100;/*<!Positive limit for the set point.*/
		float32 spNegMax = 100;/*<!Negative limit for the set point.*/
		parameters_t params;
		float32 integral;
		float32 lastError;
	};


	class MusclePlugin{

	public:
		MusclePlugin();

        ////////////////////
		/// \brief The Init function.
		///
		/// This function initializes the plugin.
		/// \param[in] myoMuscle contains info about via points, motor, gear, spindle and see of the muscles
		void Init(MyoMuscleInfo &myoMuscle);
		void Update(ros::Time &time, ros::Duration &period,
                    vector<math::Vector3> &prevForcePoints,
                    vector<math::Vector3> &nextForcePoints,
                    vector<math::Vector3> &prevForce,
					vector<math::Vector3> &nextForce);
		string name;
		map<string,vector<IViaPoints>> viaPoints;
		map<string,math::Pose> linkPose;
		PIDcontroller pid;
		double cmd = 0;
	private:
		event::ConnectionPtr connection;
		common::Time prevUpdateTime;
		physics::ModelPtr model;
		std::vector<physics::LinkPtr> links;

		IActuator::state_type x;
		ITendon tendon;
		IActuator actuator;

		double actuatorForce;

		transport::NodePtr node;
		transport::PublisherPtr visPub;

//		msgs::Visual visualMsg[linkNumber - 1];

    /*
    private: void updateObstacleOrigins();
    private: void ComputeStepTime();
    private: void ApplyForce();

    private: common::Time controllerPrevUpdateTime;
    private: common::Time currTime;
    private: common::Time stepTime;
    */
	};

}

