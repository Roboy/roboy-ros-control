#ifndef _GAZEBO_ACTUATOR_PLUGIN_HH_
#define _GAZEBO_ACTUATOR_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"
#include <sstream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <map>
#include <stdio.h>
#include <algorithm> 
#include <boost/numeric/odeint.hpp>
#include <vector>

 /// \brief Model the electromagnetic actuator, which consists of the brushed DC 
 /// motor with permanent magnets, that converts the input voltage into the angular velocity and torque of the motor 
 /// shaft, a gear, that transforms the velocity and torque from the motor, as well as a spindle
 /// that translates the angular velocity of the gear shaft to the linear velocity of
 /// the tendon and the muscle force into the load torque. 

namespace gazebo
{
  using namespace std;
  using namespace boost::numeric::odeint; 


  // ////////////////////////////////////////
  //   /// \brief Describes the system of differential equations with x[0] corresponding
  //   /// to  motor's electric current and x[1] - to spindle angular velocity
  //   /// \param[in] x  state vector
  //   /// \param[in] dx state vector
  // void DiffModel(const state_type &x , state_type &dxdt , const double /* t */);

  class ActuatorPlugin : public ModelPlugin
  {
     typedef std::vector< double > state_type;
    

    public: struct Motor
    {
      float current;
      float torqueConst;
      float resistance;
      float inductance;
      float voltage;
      float BEMFConst; // back electromagnetic force constant
      float inertiaMoment;
    };

    public: struct Gear
    {
      float inertiaMoment;
      float ratio;
      float efficiency; // gear efficciency
    };

    public: struct Spindle
    {
      float angVel; // angular velocity of the spindle
      float radius;
    };

    ////////////////////////////////////////
    /// \brief Approximates gear's velocity
    /// according to the direction of the rotation of the gear, i.e. 
    /// eta or 1/eta
    /// \return Approximated value for gear efficiency
    public: static float EfficiencyApproximation();
    // private: void DiffModel(const state_type &x , state_type &dxdt , const double /* t */);
   
    private: void ComputeDynamics();

    public: ActuatorPlugin();

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    public: void Init();

    private: void OnUpdate();

    private: event::ConnectionPtr connection;
    private: common::Time prevUpdateTime;
    private: physics::ModelPtr model;
  };
  
}

#endif
