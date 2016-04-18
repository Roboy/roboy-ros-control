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

class ForceController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
	public:
		ForceController(){

		};

		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
		{
			// get joint name from the parameter server
			if (!n.getParam("joint_name", joint_name)){
				ROS_ERROR("Could not find joint name");
				myStatus = ControllerState::UNDEFINED;
				return false;
			}
			n.getParam("id", statusMsg.id);
			ROS_INFO("ForceController %d for %s initialized", statusMsg.id, joint_name.c_str());
			joint = hw->getHandle(joint_name);  // throws on failure
			trajectory_srv = n.advertiseService("/roboy/trajectory_"+joint_name, &ForceController::trajectoryPreprocess, this);
			steer_sub = n.subscribe("/roboy/steer",1000, &ForceController::steer, this);
			status_pub = n.advertise<common_utilities::ControllerState>("/roboy/status_"+joint_name, 1000);
			myStatus = ControllerState::INITIALIZED;
			// wait for GUI subscriber
			while(status_pub.getNumSubscribers()==0)
				ROS_INFO_THROTTLE(1,"ForceController %s waiting for subscriber", joint_name.c_str());
			statusMsg.state = myStatus;
			status_pub.publish(statusMsg);
			// initialize spline to zero
			vector<double> initial_zero_x = {0,1,2}, initial_zero_y = {0,0,0};
			spline_trajectory.set_points(initial_zero_x,initial_zero_y);
			return true;
		}

		void update(const ros::Time& time, const ros::Duration& period)
		{
			float eff = joint.getEffort();

			if(steered == PLAY_TRAJECTORY) {
				dt += period.nsec/1000000;
				if (dt<trajectory_duration) {
					setpoint = spline_trajectory(dt);
				}else{
					myStatus = TRAJECTORY_DONE;
					statusMsg.state = myStatus;
					steered = STOP_TRAJECTORY;
					status_pub.publish(statusMsg);
				}
				joint.setCommand(setpoint);
			}else if(steered == STOP_TRAJECTORY){
				if(time.sec%10 == 0)
					status_pub.publish(statusMsg);
			}
		}

		void steer(const common_utilities::Steer::ConstPtr& msg){
			switch (msg->steeringCommand){
				case STOP_TRAJECTORY:
					dt = 0;
					steered = STOP_TRAJECTORY;
					myStatus = TRAJECTORY_READY;
					ROS_INFO("%s received steering STOP", joint_name.c_str());
					break;
				case PLAY_TRAJECTORY:
					steered = PLAY_TRAJECTORY;
					myStatus = TRAJECTORY_PLAYING;
					ROS_INFO("%s received steering PLAY", joint_name.c_str());
					break;
				case PAUSE_TRAJECTORY:
					if (steered==PAUSE_TRAJECTORY) {
						steered = PLAY_TRAJECTORY;
						myStatus = TRAJECTORY_PLAYING;
					}else {
						steered = PAUSE_TRAJECTORY;
						myStatus = TRAJECTORY_READY;
					}
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
		ros::Publisher  status_pub;
		tk::spline spline_trajectory;
		double trajectory_duration = 0;
		int8_t myStatus = UNDEFINED;
		int8_t steered = STOP_TRAJECTORY;
		std_msgs::Float32 msg;
		int32_t dt = 0;
		common_utilities::ControllerState statusMsg;
		bool trajectoryPreprocess(common_utilities::SetTrajectory::Request& req,
									common_utilities::SetTrajectory::Response& res){
			steered = STOP_TRAJECTORY;
			myStatus = PREPROCESS_TRAJECTORY;
			statusMsg.state = myStatus;
			status_pub.publish(statusMsg);

			trajectory_duration = req.trajectory.waypoints.size()*req.trajectory.samplerate;
			ROS_INFO("New trajectory [%d elements] at sampleRate %f, duration %f",
					 (int)req.trajectory.waypoints.size(), req.trajectory.samplerate, trajectory_duration);
			if(!req.trajectory.waypoints.empty()) {
				vector<double> x,y;
				for(uint i=0; i<req.trajectory.waypoints.size(); i++){
					x.push_back(i*req.trajectory.samplerate);
					y.push_back(req.trajectory.waypoints[i]);
				}

				spline_trajectory.set_points(x,y);
				myStatus = ControllerState::TRAJECTORY_READY;
				statusMsg.state = myStatus;
				status_pub.publish(statusMsg);
				dt = 0;
				res.state = myStatus;
				return true;
			}else{
				myStatus = ControllerState::TRAJECTORY_FAILED;
				statusMsg.state = myStatus;
				status_pub.publish(statusMsg);
				res.state = myStatus;
				dt = 0;
				trajectory_duration = 0;
				return false;
			}
		}
};
PLUGINLIB_EXPORT_CLASS(ForceController, controller_interface::ControllerBase);

