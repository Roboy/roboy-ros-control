#include "roboy.hpp"

void update(controller_manager::ControllerManager *cm){
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10);
    while(ros::ok()){
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        cm->update(time,period);
        rate.sleep();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "roboy");

    Roboy robot;

    controller_manager::ControllerManager cm(&robot);

    // we need an additional update thread, otherwise the controllers won't switch
    thread update_thread(update, &cm);

    ROS_INFO("STARTING ROBOY MAIN LOOP...");
    
    robot.main_loop(&cm);

    update_thread.join();

    ROS_INFO("TERMINATING...");

    return 0;
}

