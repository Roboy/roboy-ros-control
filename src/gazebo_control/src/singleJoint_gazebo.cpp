#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <controller_manager/controller_manager.h>
#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP

class MyRobot : public gazebo_ros_control::RobotHWSim
{
    public:
	MyRobot(){ 
	    // connect and register the joint state interface
	    hardware_interface::JointStateHandle state_handle_a("A", &pos, &vel, &eff);
	    jnt_state_interface.registerHandle(state_handle_a);
	    
	    registerInterface(&jnt_state_interface);
	    
	    // connect and register the joint position interface
	    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd);
	    jnt_pos_interface.registerHandle(pos_handle_a);
	    
	    registerInterface(&jnt_pos_interface);
	};
        
        bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
            gazebo::physics::ModelPtr parent_model, const urdf::Model *const urdf_model,
            std::vector<transmission_interface::TransmissionInfo> transmissions){
            
        };
        	
	void readSim(ros::Time time, ros::Duration period){
            LOG(INFO) << "hehe";
        };
	void writeSim(ros::Time time, ros::Duration period){
            LOG(INFO) << "hihi";
        };
	
	private:
	    hardware_interface::JointStateInterface jnt_state_interface;
	    hardware_interface::PositionJointInterface jnt_pos_interface;
	    hardware_interface::EffortJointInterface jnt_eff_interface;
	    double cmd;
	    double pos;
	    double vel;
	    double eff;
	    ros::Time prevTime;
};


int main(int argc, char* argv[])
{
    START_EASYLOGGINGPP(argc, argv);
    // Load configuration from file
    el::Configurations conf("/home/letrend/catkin_ws/logging.conf");
    // Actually reconfigure all loggers instead
    el::Loggers::reconfigureAllLoggers(conf);
    
    ros::init(argc, argv, "singleJoint");
    
    MyRobot robot;
    controller_manager::ControllerManager cm(&robot);
        
    // this is for asyncronous ros callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Control loop
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0);
    
    while (ros::ok())
    {
	const ros::Time	    time = ros::Time::now();
	const ros::Duration period = time - prev_time;
	
	robot.readSim(time, period);
	cm.update(time, period);
	robot.writeSim(time, period);
	
	rate.sleep();
    }
    return 0;
}
