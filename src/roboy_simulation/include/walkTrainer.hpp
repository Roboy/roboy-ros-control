// std
#include <cstdlib>
#include <iostream>
#include <thread>
// boost
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>
// ros
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

using namespace gazebo;
using namespace std;

class WalkTrainer{
public:
    WalkTrainer();
    ~WalkTrainer();
    void initializeWorlds(uint numberOfWorlds);
    void simulate();
    transport::NodePtr node;
    gazebo::transport::PublisherPtr serverControlPub, resetPub;
    gazebo::transport::SubscriberPtr worldModSub;
    vector<gazebo::physics::WorldPtr> world;
private:
    bool resetWorld(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);

    ros::NodeHandlePtr nh;
    ros::ServiceServer reset_world_srv;
};

void OnWorldModify(ConstWorldModifyPtr &_msg);
int main(int _argc, char **_argv);