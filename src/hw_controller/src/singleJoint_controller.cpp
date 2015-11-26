#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace position_controllers_ns{

class PositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
	PositionController(){};

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
	printf("%s update, pos: %f\n", joint_name.c_str(), pos);
	double error = setpoint_ - joint_.getPosition();
	joint_.setCommand(error*gain_);
    }
    
    void starting(const ros::Time& time) { ROS_INFO("starting controller for %s, gain: %f, setpoint: %f", joint_name.c_str(),gain_,setpoint_);}
    void stopping(const ros::Time& time) { ROS_INFO("stopping controller for %s", joint_name.c_str());}
    
private:
    hardware_interface::JointHandle joint_;
    static const double gain_ = 1.25;
    static const double setpoint_ = 3.00;
    std::string joint_name;
};
PLUGINLIB_EXPORT_CLASS(position_controllers_ns::PositionController, controller_interface::ControllerBase);
}//namespace
