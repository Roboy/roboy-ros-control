#pragma once

// ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
// messages
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include "roboy_simulation/Tendon.h"
#include "roboy_simulation/VisualizationControl.h"
#include "roboy_simulation/ForceTorque.h"
#include "roboy_simulation/LegState.h"
#include "roboy_simulation/ControllerParameters.h"
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// common definitions
#include "CommonDefinitions.h"
// muscle plugin
#include "IMuscle.hpp"
#include "helperClasses.hpp"
#include "controllerParameters.hpp"

using namespace gazebo;
using namespace std;

class WalkVisualization{
public:
    WalkVisualization();

    void visualization_control(const roboy_simulation::VisualizationControl::ConstPtr &msg);

    void publishTendon(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles);

    void publishCOM(math::Vector3 *center_of_mass);

    void publishForce(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles);

    void publishMomentArm(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles);

    void publishModel(vector<string> &link_names, physics::ModelPtr parent_model);

    void publishSimulationState(ControllerParameters &params, gazebo::common::Time gz_time_now);

    void publishStateMachineParameters(math::Vector3 *center_of_mass,
                                       math::Vector3 *foot_sole_global,
                                       CoordSys hip_CS, ControllerParameters &params);

    void publishLegState(LEG_STATE *leg_state);

    void publishCoordinateSystems(physics::LinkPtr parent_link, ros::Time time, bool child_link=false);
protected:
    ros::NodeHandlePtr nh;
    int ID;
    uint message_counter;
    bool visualizeTendon = false, visualizeCOM = false, visualizeForce = false, visualizeMomentArm = false,
            visualizeMesh = false, visualizeStateMachineParameters = false, visualizeForceTorqueSensors = false;
    ros::Publisher marker_visualization_pub;
private:
    ros::Publisher leg_state_pub, simulation_state_pub;
    ros::Subscriber visualization_control_sub;
    tf::TransformBroadcaster tf_broadcaster;
};