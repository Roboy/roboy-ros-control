#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "std_msgs/Float32.h"
#include <common_utilities/ControllerState.h>
#include <common_utilities/Steer.h>
#include <common_utilities/SetTrajectory.h>
#include <CommonDefinitions.h>
#include "timer.hpp"

using namespace std;

class PositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
    public:
	PositionController(){
            trajectory.push_back(0);
        };
        
        bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
        {
			// get joint name from the parameter server
			if (!n.getParam("joint_name", joint_name)){
				ROS_ERROR("Could not find joint name");
				myStatus = ControllerState::UNDEFINED;
				return false;
			}
			n.getParam("id", statusMsg.id);
			ROS_INFO("PositionController %d for %s initialized", statusMsg.id, joint_name.c_str());
			joint_ = hw->getHandle(joint_name);  // throws on failure
			trajectory_srv = n.advertiseService("/roboy/trajectory_"+joint_name, &PositionController::trajectoryPreprocess, this);
			steer_sub = n.subscribe("/roboy/steer",1000, &PositionController::steer, this);
			status_pub = n.advertise<common_utilities::ControllerState>("/roboy/status_"+joint_name, 1000);
			trajectory_pub = n.advertise<std_msgs::Float32>("/roboy/trajectory_"+joint_name+"/plot",1000);
			myStatus = ControllerState::INITIALIZED;
			// this pause seems to be necessary for successful communication with GUI
			ros::Duration d(1);
			d.sleep();
			statusMsg.state = myStatus;
			status_pub.publish(statusMsg);
			return true;
		}

		void update(const ros::Time& time, const ros::Duration& period)
		{
			float pos = joint_.getPosition();
			msg.data = pos;
			trajectory_pub.publish(msg);

			if(steered == PLAY_TRAJECTORY) {
				if (fabs(pos - trajectory[trajpos]) < 0.2 && trajpos < trajectory.size() - 1) {
					statusMsg.state = myStatus;
					status_pub.publish(statusMsg);
					trajpos++;
				}
				if (trajpos == trajectory.size() - 1) {
					setpoint_ = trajectory[trajpos];
					statusMsg.state = myStatus;
					status_pub.publish(statusMsg);
				}
			}else if(steered == STOP_TRAJECTORY) {
				trajpos = 0;
			}
			joint_.setCommand(trajectory[trajpos]);
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
				case PAUSE_TRAJECTORY:
					if (steered==PAUSE_TRAJECTORY) {
						steered = PLAY_TRAJECTORY;
						myStatus = TRAJECTORY_PLAYING;
					}else {
						steered = PAUSE_TRAJECTORY;
						myStatus = TRAJECTORY_READY;
					}
					ROS_INFO("%s received steering PAUSE", joint_name.c_str());
					break;
			}
			statusMsg.state = myStatus;
			status_pub.publish(statusMsg);
		}

		void starting(const ros::Time& time) { ROS_INFO("controller started for %s", joint_name.c_str());}
		void stopping(const ros::Time& time) { ROS_INFO("controller stopped for %s", joint_name.c_str());}

	private:
		hardware_interface::JointHandle joint_;
		double gain_ = 1.25; // not used
		double setpoint_ = 0;
		std::string joint_name;
		ros::NodeHandle n;
		ros::ServiceServer trajectory_srv;
		ros::Subscriber steer_sub;
		ros::Publisher trajectory_pub, status_pub;
		std::vector<float> trajectory;
		uint trajpos = 0;
		int8_t myStatus = UNDEFINED;
		int8_t steered = STOP_TRAJECTORY;
		vector<float> dt;
		vector<float> positions;
		std_msgs::Float32 msg;
		common_utilities::ControllerState statusMsg;
		Timer timer;
		bool trajectoryPreprocess(common_utilities::SetTrajectory::Request& req,
									common_utilities::SetTrajectory::Response& res){
			steered = STOP_TRAJECTORY;
			myStatus = PREPROCESS_TRAJECTORY;
			statusMsg.state = myStatus;
			status_pub.publish(statusMsg);

			ROS_INFO("New trajectory [%d elements] at sampleRate %d",
					 (int)req.trajectory.waypoints.size(), req.trajectory.samplerate);
			if(!req.trajectory.waypoints.empty()) {
				trajectory = req.trajectory.waypoints;
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
PLUGINLIB_EXPORT_CLASS(PositionController, controller_interface::ControllerBase);

