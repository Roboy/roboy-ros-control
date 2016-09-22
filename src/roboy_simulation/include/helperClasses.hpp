#pragma once

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// ros messages
#include "roboy_simulation/ControllerParameters.h"
// common definitions
#include "CommonDefinitions.h"
#include "controllerParameters.hpp"

using namespace std;

class CoordinateSystem{
public:
    CoordinateSystem(gazebo::physics::LinkPtr link):m_link(link){};
    void Update();
    void UpdateHeading();
    gazebo::math::Vector3 origin, X, Y, Z;
    gazebo::math::Quaternion rot;
private:
    gazebo::physics::LinkPtr m_link;
};

typedef boost::shared_ptr<CoordinateSystem> CoordSys;

void controllerParametersToMessage(ControllerParameters &params,
                                   roboy_simulation::ControllerParameters &msg);
void messageTocontrollerParameters(const roboy_simulation::ControllerParameters::ConstPtr &msg,
                                   ControllerParameters &params);
void messageTocontrollerParameters(roboy_simulation::ControllerParameters &msg,
                                   ControllerParameters &params);