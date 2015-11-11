#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class MyRobot : public hardware_interface::RobotHW
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
	    // Read actuator state from hardware...
	    //	    printf("read actuator state\n");
	};
	
	void write(){
	    // Send actuator command to hardware...
	    //	    printf("write actuator state\n");
	};
	
	private:
	    hardware_interface::JointStateInterface jnt_state_interface;
	    hardware_interface::PositionJointInterface jnt_pos_interface;
	    double cmd;
	    double pos;
	    double vel;
	    double eff;
	    ros::Time prevTime;
};


int main(int argc, char* argv[])
{
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