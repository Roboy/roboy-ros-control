#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <gazebo_msgs/LinkStates.h>
#include <controller_manager/controller_manager.h>

class MyRobot : public gazebo_ros_control::RobotHWSim
{
    public:
	MyRobot(){ 
	    // connect and register the joint state interface
	    hardware_interface::JointStateHandle state_handle_a("link1", &pos, &vel, &eff);
	    jnt_state_interface.registerHandle(state_handle_a);
	    
	    registerInterface(&jnt_state_interface);
	    
	    // connect and register the joint position interface
	    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("link1"), &cmd);
	    jnt_pos_interface.registerHandle(pos_handle_a);
	    
	    registerInterface(&jnt_pos_interface);
            
            n.subscribe("/gazebo/link_states", 1, &MyRobot::linkStateCallback, this);
	};
        
        bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
            gazebo::physics::ModelPtr parent_model, const urdf::Model *const urdf_model,
            std::vector<transmission_interface::TransmissionInfo> transmissions){
            ROS_INFO("init_sim");
            
        };
        
        void linkStateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){
            ROS_INFO("link state");
//            for(uint i=0;i<msg->name.size();i++){
//                ROS_INFO(msg->name[i].c_str());
//            }
        };
        	
        void readSim(ros::Time time, ros::Duration period){
            ROS_INFO("hehe");
        };
        
        void writeSim(ros::Time time, ros::Duration period){
            ROS_INFO("hihi");
        };
        
        void eStopActive(const bool active){
            ROS_INFO("THÄHÄ");
        }
	
	private:
	    hardware_interface::JointStateInterface jnt_state_interface;
	    hardware_interface::PositionJointInterface jnt_pos_interface;
	    hardware_interface::EffortJointInterface jnt_eff_interface;
	    double cmd;
	    double pos;
	    double vel;
	    double eff;
	    ros::Time prevTime;
            ros::NodeHandle n;
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "singleJoint_gazebo");
    
    MyRobot robot;
    controller_manager::ControllerManager cm(&robot);
        
    // this is for asyncronous ros callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Control loop
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(0.5);
    
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
