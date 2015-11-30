#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "FlexRayHardwareInterface.hpp"

class MyRobot : public hardware_interface::RobotHW
{
    public:
	MyRobot(){ 
            // allocate corresponding control arrays (4 motors can be connected to each ganglion)
            ROS_INFO("%d ganglions are connected via flexray", flexray.numberOfGanglionsConnected);
            cmd = new double[flexray.numberOfGanglionsConnected*4];
            pos = new double[flexray.numberOfGanglionsConnected*4];
            vel = new double[flexray.numberOfGanglionsConnected*4];
            eff = new double[flexray.numberOfGanglionsConnected*4];
            
            char motorname[10];
            int i = 0;
            
            for (uint ganglion=0;ganglion<flexray.numberOfGanglionsConnected;ganglion++){ 
                // four motors can be connected to each ganglion
                for (uint motor=0;motor<4;motor++){ 
                    sprintf(motorname, "motor%d", i);
                    // connect and register the joint state interface
                    hardware_interface::JointStateHandle state_handle(motorname, &pos[i], &vel[i], &eff[i]);
                    jnt_state_interface.registerHandle(state_handle);
                    
                    // connect and register the joint position interface
                    hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(motorname), &cmd[i]);
                    jnt_pos_interface.registerHandle(pos_handle);
                    
                    i++;
                }
            }
            
            registerInterface(&jnt_state_interface);
            registerInterface(&jnt_pos_interface);
            
	    ROS_INFO("Hardware interface initialized");
            std::string str;
            std::vector<std::string> resources = jnt_pos_interface.getNames();
	    for(uint i=0; i<resources.size();i++){
                str.append(resources[i]);
                str.append(" ");
	    }
	    ROS_INFO("Resources registered to this hardware interface:\n%s", str.c_str());
            ROS_INFO("Waiting for controller");
	};
        
        ~MyRobot(){
            delete[] cmd;
            delete[] pos;
            delete[] vel;
            delete[] eff;
        }
	
	void read(){
            ROS_DEBUG("read");
#ifdef HARDWARE
            flexray.exchangeData();
#endif
            uint i = 0;
            for (uint ganglion=0;ganglion<flexray.numberOfGanglionsConnected;ganglion++){ 
                // four motors can be connected to each ganglion
                for (uint motor=0;motor<4;motor++){ 
                    pos[i] = flexray.GanglionData[ganglion].muscleState[motor].actuatorPos*flexray.controlparams.radPerEncoderCount;
                    vel[i] = flexray.GanglionData[ganglion].muscleState[motor].actuatorVel*flexray.controlparams.radPerEncoderCount;
                    eff[i] = flexray.GanglionData[ganglion].muscleState[motor].tendonDisplacement; // dummy TODO: use polynomial approx
                    i++;
                }
            }
        };
	void write(){
            ROS_DEBUG("write");
            uint i = 0;
            for (uint ganglion=0;ganglion<flexray.numberOfGanglionsConnected;ganglion++){ 
                // four motors can be connected to each ganglion
                for (uint motor=0;motor<4;motor++){ 
                    if(ganglion<3)  // write to first commandframe
                        flexray.commandframe0[ganglion].sp[motor] = cmd[i];
                    else            // else write to second commandframe
                        flexray.commandframe1[ganglion].sp[motor] = cmd[i];
                    i++;
                }
            }
#ifdef HARDWARE
            flexray.updateCommandFrame();
            flexray.exchangeData();
#endif
        };
	
	private:
	    hardware_interface::JointStateInterface jnt_state_interface;
	    hardware_interface::PositionJointInterface jnt_pos_interface;
	    hardware_interface::EffortJointInterface jnt_eff_interface;
	    double *cmd;
	    double *pos;
	    double *vel;
	    double *eff;
	    ros::Time prevTime;
            FlexRayHardwareInterface flexray;
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "multiJoint");
    
    MyRobot robot;
    controller_manager::ControllerManager cm(&robot);
    
    // this is for asyncronous ros callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Control loop
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10);
    
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

