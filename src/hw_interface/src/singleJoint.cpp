#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "FlexRayHardwareInterface.hpp"
INITIALIZE_EASYLOGGINGPP

class MyRobot : public hardware_interface::RobotHW ,public FlexRayHardwareInterface 
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
	
	void read(){
            readFromFlexray();
            
            pos = GanglionData[0].muscleState[0].actuatorPos;
            vel = GanglionData[0].muscleState[0].actuatorVel;
            eff = GanglionData[0].muscleState[0].tendonDisplacement; // dummy TODO: use polynomial approx
        };
	void write(){
            commandframe[0].sp[0] = cmd;
            changeCommandFrame(commandframe,&controlparams);
            writeToFlexray();
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
	
	robot.read();
	cm.update(time, period);
	robot.write();
	
	rate.sleep();
    }
    return 0;
}
