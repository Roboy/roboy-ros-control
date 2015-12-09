#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <math.h>

namespace position_controllers_ns{
    
    class PositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
    {
    public:
	PositionController(){
            ROS_INFO("Subscribing to trajectory_advertiser");
            sub = n.subscribe("trajectory_advertiser", 1000, &PositionController::trajectoryCB, this);
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
        ros::Subscriber sub;
        std::vector<float> trajectory;
        uint trajpos = 0;
        bool reachedEndpoint = false;
        void trajectoryCB(const std_msgs::Float32MultiArray::ConstPtr& msg){
            ROS_INFO("New trajectory [%d elements]", (int)msg->data.size());
            trajectory.resize(msg->data.size());
            trajpos=0;
            reachedEndpoint = false;
            for(uint i=0;i<msg->data.size();i++){
                trajectory[i]=msg->data[i];
                ROS_INFO("%f ",trajectory[i]);
            }
            setpoint_ = trajectory[0];
        }
    };
    PLUGINLIB_EXPORT_CLASS(position_controllers_ns::PositionController, controller_interface::ControllerBase);
}//namespace
