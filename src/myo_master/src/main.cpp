#include "roboy.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "roboy");
    
    Roboy robot;

    robot.main_loop();

    return 0;
}

