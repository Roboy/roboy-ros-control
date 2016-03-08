#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "std_msgs/Float32.h"
#include <common_utilities/Status.h>
#include <common_utilities/Steer.h>
#include <common_utilities/Trajectory.h>
#include <CommonDefinitions.h>
#include "timer.hpp"

using namespace std;

class ForceController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
	public:
		ForceController(){
			trajectory.push_back(0);
		};

		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
		{
			// get joint name from the parameter server
			if (!n.getParam("joint_name", joint_name)){
				ROS_FATAL("Could not find joint name");
				myStatus = UNDEFINED;
				return false;
			}
			joint_ = hw->getHandle(joint_name);  // throws on failure
			trajectory_srv = n.advertiseService("/roboy/trajectory_"+joint_name, &ForceController::trajectoryPreprocess, this);
			steer_sub = n.subscribe("/roboy/steer",1000, &ForceController::steer, this);
			status_pub = n.advertise<common_utilities::Status>("/roboy/status_"+joint_name, 1);
			trajectory_pub = n.advertise<std_msgs::Float32>("/roboy/trajectory_"+joint_name+"/plot",1);
			myStatus = INITIALIZED;
			statusMsg.statusCode = myStatus;
			statusMsg.steered = steered;
			status_pub.publish(statusMsg);
			ROS_DEBUG("PositionController for %s initialized", joint_name.c_str());
			return true;
        }

        void update(const ros::Time& time, const ros::Duration& period)
        {
			float pos = joint_.getPosition();
			msg.data = pos;
			trajectory_pub.publish(msg);

			if(steered == PLAY_TRAJECTORY) {
				dt.push_back(timer.elapsedTime());
				positions.push_back(pos);

				if (fabs(pos - trajectory[trajpos]) < 0.2 && trajpos < trajectory.size() - 1) {
					myStatus = TRAJECTORY_PLAYING;
					statusMsg.statusCode = myStatus;
					statusMsg.steered = steered;
					status_pub.publish(statusMsg);
					trajpos++;
					setpoint_ = trajectory[trajpos];
				}

				if (trajpos == trajectory.size() - 1) {
					ROS_DEBUG_THROTTLE(1, "%s update, current pos: %f, reached endpoint of trajectory %f",
									   joint_name.c_str(), pos, trajectory[trajpos]);
					setpoint_ = trajectory[trajpos];
					myStatus = TRAJECTORY_DONE;
					statusMsg.statusCode = myStatus;
					statusMsg.steered = steered;
					status_pub.publish(statusMsg);
                    steered = STOP_TRAJECTORY;
				} else {
					ROS_DEBUG_THROTTLE(1, "%s update, current pos: %f, setpoint: %f", joint_name.c_str(), pos,
									   setpoint_);
				}
				joint_.setCommand(setpoint_);
			}
        }

		void steer(const common_utilities::Steer::ConstPtr& msg){
			switch (msg->steeringCommand){
				case STOP_TRAJECTORY:
					steered = STOP_TRAJECTORY;
					trajpos = 0;
					myStatus = TRAJECTORY_READY;
					break;
				case PLAY_TRAJECTORY:
					steered = PLAY_TRAJECTORY;
					myStatus = TRAJECTORY_PLAYING;
					break;
				case PAUSE_TRAJECTORY:
					steered = PAUSE_TRAJECTORY;
					myStatus = TRAJECTORY_READY;
					break;
				case REWIND_TRAJECTORY:
					steered = PLAY_TRAJECTORY;
					trajpos = 0;
					myStatus = TRAJECTORY_READY;
					break;
			}
			statusMsg.statusCode = myStatus;
			statusMsg.steered = steered;
			status_pub.publish(statusMsg);
		}

        void starting(const ros::Time& time) { ROS_DEBUG("starting controller for %s, gain: %f, setpoint: %f", joint_name.c_str(),gain_,setpoint_);}
        void stopping(const ros::Time& time) { ROS_DEBUG("stopping controller for %s", joint_name.c_str());}

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
			common_utilities::Status statusMsg;
            Timer timer;
            bool trajectoryPreprocess(common_utilities::Trajectory::Request& req,
                                        common_utilities::Trajectory::Response& res){
				steered = STOP_TRAJECTORY;
				myStatus = PREPROCESS_TRAJECTORY;
				statusMsg.statusCode = myStatus;
				statusMsg.steered = steered;
				status_pub.publish(statusMsg);

                ROS_DEBUG("New trajectory [%d elements] at sampleRate %d",
						 (int)req.waypoints.size(), req.samplerate);
				if(!req.waypoints.empty()) {
					trajectory = req.waypoints;
					trajpos = 0;
					myStatus = TRAJECTORY_READY;
					statusMsg.statusCode = myStatus;
					statusMsg.steered = steered;
					status_pub.publish(statusMsg);
                    timer.start();
					res.state = TRAJECTORY_READY;
					return true;
				}else{
					myStatus = TRAJECTORY_FAILED;
					statusMsg.statusCode = myStatus;
					statusMsg.steered = steered;
					status_pub.publish(statusMsg);
					res.state = TRAJECTORY_FAILED;
					return false;
				}
            }
};
PLUGINLIB_EXPORT_CLASS(ForceController, controller_interface::ControllerBase);

