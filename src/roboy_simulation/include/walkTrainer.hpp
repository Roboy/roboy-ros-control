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
#include <interactive_markers/interactive_marker_server.h>
//messages
#include <std_msgs/Int32.h>
// common definitions
#include "CommonDefinitions.h"

using namespace gazebo;
using namespace std;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server;
physics::ModelPtr modelControl;
bool paused = false;
bool slow_motion = false;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
class WalkTrainer{
public:
    WalkTrainer();
    ~WalkTrainer();
    void initializeWorlds(uint numberOfWorlds);
    void simulate();
    transport::NodePtr node;
    transport::PublisherPtr serverControlPub, resetPub;
    vector<physics::WorldPtr> world;
    vector<physics::ModelPtr> model;
    vector<int> roboyIDs;
private:
    bool resetWorld(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
    void initializeInterActiveMarkers(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                      physics::ModelPtr model, int roboyID);
    void updateID(const std_msgs::Int32::ConstPtr &msg);
    void simulationControl(const std_msgs::Int32::ConstPtr &msg);

    ros::NodeHandlePtr nh;
    ros::ServiceServer reset_world_srv;
    ros::Subscriber roboyID_sub, sim_control_sub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
};

void OnWorldModify(ConstWorldModifyPtr &_msg);
int main(int _argc, char **_argv);