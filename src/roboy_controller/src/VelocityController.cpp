#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include "common_utilities/ControllerState.h"
#include "common_utilities/Steer.h"
#include "common_utilities/SetTrajectory.h"
#include "CommonDefinitions.h"
#include "spline.h"
#include "timer.hpp"

using namespace std;

class VelocityController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
		public:
		VelocityController(){

        };

        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)
        {
			// get joint name from the parameter server
			if (!n.getParam("joint_name", joint_name)){
				ROS_ERROR("Could not find joint name");
				myStatus = ControllerState::UNDEFINED;
				return false;
			}
			n.getParam("id", statusMsg.id);
			ROS_INFO("VelocityController %d for %s initialized", statusMsg.id, joint_name.c_str());
			joint = hw->getHandle(joint_name);  // throws on failure
			trajectory_srv = n.advertiseService("/roboy/trajectory_"+joint_name, &VelocityController::trajectoryPreprocess, this);
			steer_sub = n.subscribe("/roboy/steer",1000, &VelocityController::steer, this);
			status_pub = n.advertise<common_utilities::ControllerState>("/roboy/status_"+joint_name, 1000);
			trajectory_pub = n.advertise<std_msgs::Float32>("/roboy/trajectory_"+joint_name+"/plot",1000);
			myStatus = ControllerState::INITIALIZED;
			// wait for GUI subscriber
			while(status_pub.getNumSubscribers()==0 && trajectory_pub.getNumSubscribers()==0)
				ROS_INFO_THROTTLE(1,"VelocityController %s waiting for subscriber", joint_name.c_str());
			statusMsg.state = myStatus;
			status_pub.publish(statusMsg);
			return true;
		}

		void update(const ros::Time& time, const ros::Duration& period)
		{
			float vel = joint.getVelocity();
			msg.data = vel;
			trajectory_pub.publish(msg);

			if(steered == PLAY_TRAJECTORY) {
				double dt = timer.elapsedTimeMilliSeconds();
				if (dt<trajectory_duration) {
					setpoint = spline_trajectory(dt);
				}else{
					myStatus = TRAJECTORY_DONE;
					statusMsg.state = myStatus;
					steered = STOP_TRAJECTORY;
					status_pub.publish(statusMsg);
				}
				joint.setCommand(setpoint);
			}
		}

		void steer(const common_utilities::Steer::ConstPtr& msg){
			switch (msg->steeringCommand){
				case STOP_TRAJECTORY:
					steered = STOP_TRAJECTORY;
					myStatus = TRAJECTORY_READY;
					ROS_INFO("%s received steering STOP", joint_name.c_str());
					break;
				case PLAY_TRAJECTORY:
					steered = PLAY_TRAJECTORY;
					myStatus = TRAJECTORY_PLAYING;
					ROS_INFO("%s received steering PLAY", joint_name.c_str());
					break;
			}
			statusMsg.state = myStatus;
			status_pub.publish(statusMsg);
		}

		void starting(const ros::Time& time) { ROS_INFO("controller started for %s", joint_name.c_str());}
		void stopping(const ros::Time& time) { ROS_INFO("controller stopped for %s", joint_name.c_str());}

	private:
		hardware_interface::JointHandle joint;
		double setpoint = 0;
		string joint_name;
		ros::NodeHandle n;
		ros::ServiceServer trajectory_srv;
		ros::Subscriber steer_sub;
		ros::Publisher trajectory_pub, status_pub;
		tk::spline spline_trajectory;
		double trajectory_duration;
		uint trajpos = 0;
		int8_t myStatus = UNDEFINED;
		int8_t steered = STOP_TRAJECTORY;
		std_msgs::Float32 msg;
		common_utilities::ControllerState statusMsg;
		Timer timer;
		bool trajectoryPreprocess(common_utilities::SetTrajectory::Request& req,
									common_utilities::SetTrajectory::Response& res){
			steered = STOP_TRAJECTORY;
			myStatus = PREPROCESS_TRAJECTORY;
			statusMsg.state = myStatus;
			status_pub.publish(statusMsg);

			ROS_INFO("New trajectory [%d elements] at sampleRate %f",
					 (int)req.trajectory.waypoints.size(), req.trajectory.samplerate);
			if(!req.trajectory.waypoints.empty()) {
				vector<double> x,y;
				for(uint i=0; i<req.trajectory.waypoints.size(); i++){
					x.push_back(i*req.trajectory.samplerate);
					y.push_back(req.trajectory.waypoints[i]);
				}
				trajectory_duration = x.back();
				spline_trajectory.set_points(x,y);
				trajpos = 0;
				myStatus = ControllerState::TRAJECTORY_READY;
				statusMsg.state = myStatus;
				status_pub.publish(statusMsg);
				timer.start();
				res.state = myStatus;
				return true;
			}else{
				myStatus = ControllerState::TRAJECTORY_FAILED;
				statusMsg.state = myStatus;
				status_pub.publish(statusMsg);
				res.state = myStatus;
				return false;
			}
		}
};
PLUGINLIB_EXPORT_CLASS(VelocityController, controller_interface::ControllerBase);

