#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "../../common_utilities/include/CommonDefinitions.h"
#include <common_utilities/Status.h>
#include <common_utilities/Steer.h>
#include <common_utilities/Trajectory.h>
#include <CommonDefinitions.h>
#include <math.h>
//#include <ncurses.h>

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
                ROS_FATAL("Could not find joint name");
                return false;
            }
            joint_ = hw->getHandle(joint_name);  // throws on failure
            trajectory_srv = n.advertiseService("/roboy/trajectory_"+joint_name, &PositionController::trajectoryPreprocess, this);
			status_srv = n.advertiseService("/roboy/status_"+joint_name, &PositionController::statusService, this);
			myStatus = STATUS::INITIALIZED;
			ROS_INFO("PositionController for %s initialized", joint_name.c_str());
            return true;
        }
        
        void update(const ros::Time& time, const ros::Duration& period)
        {
            float pos = joint_.getPosition();

			setpoint_ = trajectory[0];

            joint_.setCommand(setpoint_);
        }

		bool statusService(common_utilities::Status::Request &req,
							common_utilities::Status::Response &res){
			res.statusCode = myStatus;
			return true;
		}

        void starting(const ros::Time& time) { ROS_INFO("starting controller for %s, gain: %f, setpoint: %f", joint_name.c_str(),gain_,setpoint_);}
        void stopping(const ros::Time& time) { ROS_INFO("stopping controller for %s", joint_name.c_str());}
        
        private:
            hardware_interface::JointHandle joint_;
            double gain_ = 1.25; // not used
            double setpoint_ = 0;
            std::string joint_name;
            ros::NodeHandle n;
            ros::ServiceServer trajectory_srv, status_srv;
            ros::Subscriber steer_sub;
            std::vector<float> trajectory;
            uint trajpos = 0;
			int8_t myStatus = 0;
            bool trajectoryPreprocess(common_utilities::Trajectory::Request& req,
                                        common_utilities::Trajectory::Response& res){
                ROS_INFO("New trajectory [%d elements] in controlMode %d at sampleRate %d",
						 (int)req.waypoints.size(), req.controlmode, req.samplerate);
                trajectory = req.waypoints;
                return true;
            }
};
PLUGINLIB_EXPORT_CLASS(PositionController, controller_interface::ControllerBase);

