#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include "common_utilities/ControllerState.h"
#include "common_utilities/Steer.h"
#include "CommonDefinitions.h"
#include <mutex>

using namespace std;

class MuscleActivityController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    MuscleActivityController();

    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);

    void update(const ros::Time& time, const ros::Duration& period);

    void steer(const common_utilities::Steer::ConstPtr& msg);

    void starting(const ros::Time& time) { ROS_INFO("controller started for %s", joint_name.c_str());}
    void stopping(const ros::Time& time) { ROS_INFO("controller stopped for %s", joint_name.c_str());}

private:
    void newSetpoint( const std_msgs::Float32::ConstPtr& msg );

    hardware_interface::JointHandle joint;
    double setpoint = 0;
    string joint_name;
    ros::NodeHandle n;
    ros::Subscriber steer_sub, setPoint_sub;
    ros::Publisher  status_pub, setPoint_pub;
    int8_t myStatus = UNDEFINED;
    int8_t steered = PLAY_TRAJECTORY;
    std_msgs::Float32 eff_msg;
    float dt = 0;
    common_utilities::ControllerState statusMsg;
    mutex mux;
};

