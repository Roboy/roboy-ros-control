#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include "common_utilities/ControllerState.h"
#include "common_utilities/Steer.h"
#include "common_utilities/Trajectory.h"
#include "CommonDefinitions.h"
#include "spline.h"
#include "timer.hpp"

using namespace std;

class PositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
    public:
	PositionController(){

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
			joint = hw->getHandle(joint_name);  // throws on failure
			trajectory_sub = n.subscribe("/roboy/trajectory_"+joint_name, 1, &PositionController::trajectoryPreprocess, this);
			steer_sub = n.subscribe("/roboy/steer",1000, &PositionController::steer, this);
			status_pub = n.advertise<common_utilities::ControllerState>("/roboy/status_"+joint_name, 1000);
			trajectory_pub = n.advertise<std_msgs::Float32>("/roboy/trajectory_"+joint_name+"/pos", 1000);
			myStatus = ControllerState::INITIALIZED;
			// wait for GUI subscriber
//			while(status_pub.getNumSubscribers()==0)
//				ROS_INFO_THROTTLE(1,"PositionController %s waiting for subscriber", joint_name.c_str());
			statusMsg.state = myStatus;
			status_pub.publish(statusMsg);
			// initialize spline to current position (spline needs at least three values)
			setpoint = joint.getPosition();
			vector<double> initial_zero_x = {0,1,2}, initial_zero_y = {setpoint,setpoint,setpoint};
			spline_trajectory.set_points(initial_zero_x,initial_zero_y);
			return true;
		}

		void update(const ros::Time& time, const ros::Duration& period)
		{
			double pos = joint.getPosition();
			pos_msg.data = pos;
			trajectory_pub.publish(pos_msg);

			if(steered == PLAY_TRAJECTORY) {
				dt += period.nsec*1e-6f;
//                ROS_INFO("%f", dt );
				if (dt<trajectory_duration) {
					setpoint = spline_trajectory(dt);
                    if(setpoint<0.0)
                        setpoint = 0.0;
				}else{
                    setpoint = 0.0;
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
		ros::Subscriber steer_sub, trajectory_sub;
		ros::Publisher  status_pub, trajectory_pub;
		tk::spline spline_trajectory;
		double trajectory_duration = 0;
		int8_t myStatus = UNDEFINED;
		int8_t steered = STOP_TRAJECTORY;
		std_msgs::Float32 pos_msg;
		float dt = 0;
		common_utilities::ControllerState statusMsg;
		void trajectoryPreprocess(const common_utilities::Trajectory::ConstPtr& msg ){
			steered = STOP_TRAJECTORY;
			myStatus = PREPROCESS_TRAJECTORY;
			statusMsg.state = myStatus;
			status_pub.publish(statusMsg);

			trajectory_duration = msg->waypoints.size()*msg->samplerate;
			ROS_INFO("New trajectory [%d elements] at sampleRate %f, duration %f",
					 (int)msg->waypoints.size(), msg->samplerate, trajectory_duration);
			if(!msg->waypoints.empty()) {
				vector<double> x,y;
				for(uint i=0; i<msg->waypoints.size(); i++){
                    x.push_back(i*msg->samplerate);
                    if(msg->waypoints[i]!=FLT_MAX) {
                        y.push_back(msg->waypoints[i]);
                        cout << msg->waypoints[i] << " ";
                    }else{
                        y.push_back(0.0);
                        cout << msg->waypoints[i] << " ";
                    }
				}
				cout << endl;
				spline_trajectory.set_points(x,y);
				myStatus = ControllerState::TRAJECTORY_READY;
				statusMsg.state = myStatus;
				status_pub.publish(statusMsg);
				dt = 0;
			}else{
				myStatus = ControllerState::TRAJECTORY_FAILED;
				statusMsg.state = myStatus;
				status_pub.publish(statusMsg);
				dt = 0;
				trajectory_duration = 0;
			}
		}
};
PLUGINLIB_EXPORT_CLASS(PositionController, controller_interface::ControllerBase);

