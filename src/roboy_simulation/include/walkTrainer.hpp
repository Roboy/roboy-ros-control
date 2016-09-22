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
// ros messages
#include <std_msgs/Int32.h>
#include "roboy_simulation/ControllerParameters.h"
#include "roboy_simulation/UpdateControllerParameters.h"
#include "roboy_simulation/Energies.h"
// common definitions
#include "CommonDefinitions.h"
#include "controllerParameters.hpp"
#include "helperClasses.hpp"
// libcmaes
#include "cmaes.h"

using namespace gazebo;
using namespace std;
using namespace libcmaes;

#define POPULATION_SIZE 1


boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server;
physics::ModelPtr modelControl;
bool paused = false;
bool slow_motion = false;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
class WalkTrainer : public CMAStrategy<CovarianceUpdate>{
public:
    WalkTrainer(FitFunc &func, CMAParameters<> &parameters);
    ~WalkTrainer();
    /**
     * initializes numberOfWorlds worlds and populates them with the legs
     * @param numberOfWorlds
     */
    void initializeWorlds(uint numberOfWorlds);
    void simulate();
    /** Initializes ControllerParameters */
    void initializeControllerParameters(ControllerParameters &params, physics::ModelPtr parent_model);
    bool resetWorld(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
    void initializeInterActiveMarkers(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                      physics::ModelPtr model, int roboyID);
    dMat ask();
    void eval(const dMat &candidates, const dMat &phenocandidates=dMat(0,0));
    void tell();
    bool stop();
    ControllerParameters inital_params;
private:
    void updateID(const std_msgs::Int32::ConstPtr &msg);
    void simulationControl(const std_msgs::Int32::ConstPtr &msg);
    transport::NodePtr node;
    transport::PublisherPtr serverControlPub, resetPub;

    ros::NodeHandlePtr nh;
    ros::ServiceServer reset_world_srv;
    vector<ros::ServiceClient> control_parameters_srvs, energie_srvs;
    ros::Subscriber sim_control_sub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;

    vector<int> roboyIDs;
    vector<physics::WorldPtr> world;
    vector<physics::ModelPtr> model;
    vector<ControllerParameters> controllerParams;
};

void OnWorldModify(ConstWorldModifyPtr &_msg);
int main(int _argc, char **_argv);