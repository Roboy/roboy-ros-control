#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_advertiser");
    
    ros::NodeHandle n;
    
    ros::Publisher trajectory_advertiser = n.advertise<std_msgs::Float32MultiArray>("trajectory_advertiser", 1000);
    
    while (ros::ok())
    {
        ROS_INFO("send trajectory? [y/n/new trajectory(e.g. 1 2 3 4)]");
        std::string s;
        std::getline(std::cin, s);
        if(strcmp("y",s.c_str())==0){
            ROS_INFO("sending again");
        }else if(strcmp("n",s.c_str())==0){
            ROS_FATAL("abort");
            break;
        }else{
            std_msgs::Float32MultiArray msg;
            std::istringstream iss{s};
            
            msg.data.clear();
            
            float f;
            while (iss >> f){
                msg.data.push_back(f);
            }

            trajectory_advertiser.publish(msg);

            ros::spinOnce();
        }
    }
    
    
    return 0;
}
