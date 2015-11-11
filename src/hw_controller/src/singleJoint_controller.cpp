#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace singleJoint_controller_ns{

class PositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
    {
	// get joint name from the parameter server
	std::string joint_name;
	if (!n.getParam("joint_name", joint_name)){
	    ROS_ERROR("Could not find joint name");
	    return false;
	}
	
	// get the joint object to use in the realtime loop
	joint_ = hw->getHandle(joint_name);  // throws on failure
	return true;
    }
    
    void update(const ros::Time& time, const ros::Duration& period)
    {
	double error = setpoint_ - joint_.getPosition();
	joint_.setCommand(error*gain_);
    }
    
    void starting(const ros::Time& time) { }
    void stopping(const ros::Time& time) { }
    
private:
    hardware_interface::JointHandle joint_;
    static const double gain_ = 1.25;
    static const double setpoint_ = 3.00;
};
PLUGINLIB_EXPORT_CLASS(singleJoint_controller_ns::PositionController, controller_interface::ControllerBase);
}//namespace