#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <common_utilities/Status.h>
#include <common_utilities/Steer.h>
#include <common_utilities/Trajectory.h>
#include <math.h>
//#include <ncurses.h>

class PositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
    public:
	PositionController(){
            ROS_INFO("Subscribing to /roboy/trajectory");
            status_pub = n.advertise<common_utilities::Status>("/roboy/statusResponse",1);
            trajectory.push_back(0);
        };
        
        bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
        {
            // get joint name from the parameter server
            if (!n.getParam("joint_name", joint_name)){
                ROS_FATAL("Could not find joint name");
                return false;
            }
            // get the joint object to use in the realtime loop
            joint_ = hw->getHandle(joint_name);  // throws on failure
            trajectory_srv = n.advertiseService("/roboy/trajectory_"+joint_name, &PositionController::trajectoryPreprocess, this);
            ROS_INFO("PositionController for %s initialized", joint_name.c_str());
            return true;
        }
        
        void update(const ros::Time& time, const ros::Duration& period)
        {
            float pos = joint_.getPosition();
            if(fabs(pos-trajectory[trajpos])<0.2 && trajpos < trajectory.size()-1){
                trajpos++;
                setpoint_ = trajectory[trajpos];
            }
            
            if(trajpos == trajectory.size()-1){
                ROS_INFO_THROTTLE(1,"%s update, current pos: %f, reached endpoint of trajectory %f", joint_name.c_str(), pos, trajectory[trajpos]);
                setpoint_ = trajectory[trajpos];
            }else{
                ROS_INFO_THROTTLE(1,"%s update, current pos: %f, setpoint: %f", joint_name.c_str(), pos, setpoint_);
            }
            joint_.setCommand(setpoint_);
        }
        
        void starting(const ros::Time& time) { ROS_INFO("starting controller for %s, gain: %f, setpoint: %f", joint_name.c_str(),gain_,setpoint_);}
        void stopping(const ros::Time& time) { ROS_INFO("stopping controller for %s", joint_name.c_str());}
        
        private:
            hardware_interface::JointHandle joint_;
            double gain_ = 1.25; // not used
            double setpoint_ = 0;
            std::string joint_name;
            ros::NodeHandle n;
            ros::ServiceServer trajectory_srv;
            ros::Publisher status_pub;
            std::vector<float> trajectory;
            uint trajpos = 0;
            bool trajectoryPreprocess(common_utilities::Trajectory::Request& req,
                                        common_utilities::Trajectory::Response& res){
                ROS_INFO("New trajectory [%d elements] in controlMode %d at sampleRate %d",
						 (int)req.waypoints.size(), req.controlmode, req.samplerate);
                trajectory = req.waypoints;
                return true;
            }
};
PLUGINLIB_EXPORT_CLASS(PositionController, controller_interface::ControllerBase);

