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
		vector<IViaPoints> viaPoints;
		Motor motor;
		Gear gear;
		Spindle spindle;
		SEE see;
		MyoMuscleInfo():viaPoints(){};
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
		void Update(ros::Time &time, ros::Duration &period );
		string name;
		vector<IViaPoints> viaPoints;
		double cmd = 0;
	private:
		event::ConnectionPtr connection;
		common::Time prevUpdateTime;

		IActuator::state_type x;
		ITendon tendon;
		IActuator actuator;

		double actuatorForce;

		transport::NodePtr node;
		transport::PublisherPtr visPub;

//		msgs::Visual visualMsg[linkNumber - 1];

	};

}

