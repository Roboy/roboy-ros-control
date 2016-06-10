#include <sstream>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <stdio.h>
#include <algorithm> 
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>

#include "MusclePlugin.hh"

using namespace gazebo;

//Register plugin with this simulator
GZ_REGISTER_MODEL_PLUGIN(MusclePlugin);

Motor _motor;
Gear _gear;
Spindle _spindle;
SEE _see;

math::Vector3 insertion;
math::Vector3 fixation;

float elasticForce;
float actuatorForce;

IActuator::state_type x(2);
IActuator actuator;
ITendon tendon;

float IActuator::EfficiencyApproximation()
{
	float param1 = 0.1; // defines steepness of the approxiamtion
	float param2 = 0; // defines zero crossing of the approxiamtion
	return _gear.efficiency + (1/_gear.efficiency - _gear.efficiency)*(0.5*(tanh(-param1 * _spindle.angVel * _motor.current - param2) +1));
}


void IActuator::DiffModel( const state_type &x , state_type &dxdt , const double /* t */ )
{
    //x[0] - motor electric current
    //x[1] - spindle angular velocity
	float totalIM = _motor.inertiaMoment + _gear.inertiaMoment; // total moment of inertia
    dxdt[0] = 1/_motor.inductance * (-_motor.resistance * x[0] -_motor.BEMFConst * _gear.ratio * x[1] + _motor.voltage);
    dxdt[1] = _motor.torqueConst * x[0] / (_gear.ratio * totalIM) - 
    	_spindle.radius * elasticForce / (_gear.ratio * _gear.ratio * totalIM * _gear.appEfficiency);
}

float ITendon::DotProduct(const math::Vector3 &_v1, const math::Vector3 &_v2)
{
	return _v1.x*_v2.x + _v1.y*_v2.y + _v1.z*_v2.z;
}


float ITendon::Angle(const math::Vector3 &_v1, const math::Vector3 &_v2)
{
	return acos(_v1.Dot(_v2)/_v1.GetLength()*_v2.GetLength());
}


float ITendon::ElectricMotorModel(const float _current,  const float _torqueConstant, 
					const float _spindleRadius)
{
    float motorForce;

    if (_current>=0)
    {
        motorForce=_current*_torqueConstant/_spindleRadius;
    }
    else
    {
        motorForce=0;
    }
    
	return motorForce;
}


float ITendon::ElasticElementModel(const float _length0, const float _length, float _stiffness,
							 const float _speed, const float _spindleRadius)
{
    float realTimeUpdateRate=1000;
    float windingLength = 1;//_spindleRadius*_speed/realTimeUpdateRate*timeCounter;
	float displacement;
	displacement = windingLength + _length - _length0;
    
	// gzdbg << "displacement: " 
	// 	  << displacement
	// 	  << "\n"
 //          << "windingLength: " 
	// 	  << windingLength
	// 	  << "\n";
    
    float elasticForce;

    if (displacement>=0)
    {
        elasticForce=displacement*_stiffness;
    }
    else
    {
        elasticForce=0;
    }

	//return _stiffness[0] + (displacement*_stiffness[1]) + (displacement*displacement*_stiffness[2]) + 
	//			(displacement*displacement*displacement*_stiffness[3]) ;
    //return displacement*_stiffness[0];
    return elasticForce;	 
                
}


math::Vector3 ITendon::CalculateForce(float _elasticForce, float _motorForce, 
	const math::Vector3 &_instertionP, const math::Vector3 &_fixationP)
{
	math::Vector3 diff = _fixationP - _instertionP;
    
	/*    float tendonForce;
    
    if (_elasticForce+_motorForce>=0)
    {
        tendonForce=_elasticForce+_motorForce;
    }
    else
    {
        tendonForce=0;
    }*/
    
	return diff/diff.GetLength()*(_elasticForce+_motorForce);
    
}


void MusclePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	//Store pointer to the model
	this->model = _parent;

	//Get the parameters from SDF
	gzmsg << "Reading values from SDF" << std::endl;

	if (_sdf->HasElement("motor"))
	{	
		sdf::ElementPtr motorElement = _sdf->GetElement("motor");

		if (//!motorElement->HasElement("electric_current") || 
			!motorElement->HasElement("torque_constant") ||
			!motorElement->HasElement("BEMF_constant") ||
			!motorElement->HasElement("inductance") ||
			!motorElement->HasElement("resistance") ||
			!motorElement->HasElement("inertiaMoment"))

		{
			gzwarn << "Invalid SDF: Missing required elements for motor model";
		}

		if (motorElement->HasElement("torque_constant"))
		{	
			_motor.torqueConst = motorElement->Get<float>("torque_constant");	
			gzdbg << "torque_constant " <<_motor.torqueConst<< "\n";
		}

		if (motorElement->HasElement("bemf_constant"))
		{
			_motor.BEMFConst = motorElement->Get<float>("bemf_constant");	
			gzdbg << "bemf_constant " <<_motor.BEMFConst<< "\n";
		}

		if (motorElement->HasElement("inductance"))
		{	
			_motor.inductance = motorElement->Get<float>("inductance");	
			gzdbg << "inductance " <<_motor.inductance<< "\n";
		}

		if (motorElement->HasElement("resistance"))
		{	
			_motor.resistance = motorElement->Get<float>("resistance");	
			gzdbg << "resistance " <<_motor.resistance<< "\n";
		}

		if (motorElement->HasElement("inertiaMoment"))
		{	
			_motor.inertiaMoment = motorElement->Get<float>("inertiaMoment");	
			gzdbg << "inertia " <<_motor.inertiaMoment<< "\n";
		}
	}

	if (_sdf->HasElement("gear"))
	{	
		sdf::ElementPtr gearElement = _sdf->GetElement("gear");

		if (!gearElement->HasElement("ratio") ||
			!gearElement->HasElement("efficiency") ||
			!gearElement->HasElement("inertiaMoment"))

		{
			gzwarn << "Invalid SDF: Missing required elements for gear model";
		}
		
		if (gearElement->HasElement("ratio"))
		{	
			_gear.ratio = gearElement->Get<float>("ratio");	
			gzdbg << "ratio " <<_gear.ratio<< "\n";
		}

		if (gearElement->HasElement("efficiency"))
		{
			_gear.efficiency = gearElement->Get<float>("efficiency");	
			gzdbg << "efficiency " <<_gear.efficiency<< "\n";
		}

		if (gearElement->HasElement("inertiaMoment"))
		{	
			_gear.inertiaMoment = gearElement->Get<float>("inertiaMoment");	
			gzdbg << "inertia " <<_gear.inertiaMoment<< "\n";
		}
	}


	if (_sdf->HasElement("spindle"))
	{	
		sdf::ElementPtr spindleElement = _sdf->GetElement("spindle");

		if (!spindleElement->HasElement("radius"))

		{
			gzwarn << "Invalid SDF: Missing required elements for spindle model";
		}

		if (spindleElement->HasElement("radius"))
		{	
			_spindle.radius = spindleElement->Get<float>("radius");	
			gzdbg << "radius " <<_spindle.radius<< "\n";
		}
	}

	if (_sdf->HasElement("SEE"))
	{	
		
		sdf::ElementPtr elasticElement = _sdf->GetElement("SEE");

        
		if (!elasticElement->HasElement("stiffness") ||
			 !elasticElement->HasElement("length0"))
		{
			gzwarn << "Invalid SDF: Missing required elements for series elastic element";
		}

            //stiffness
			// sdf::ElementPtr stiffnessElement = elasticElement->GetElement("stiffness");
			//float stiffness[4];

		if (elasticElement->HasElement("stiffness"))
		{
			_see.stiffness = elasticElement->Get<float>("stiffness");
		}
		

			//get floats from string
			// std::istringstream ss(stiffnessString);
			// std::copy(std::istream_iterator <float> (ss),
			// 	std::istream_iterator <float>(),
			// 	SEE.stiffness);

		if (elasticElement->HasElement("length0"))
		{	
			_see.lengthRest = elasticElement->Get<float>("length0");	
		}

	}

	this->connection = event::Events::ConnectWorldUpdateBegin(
         			 boost::bind(&MusclePlugin::OnUpdate, this));
}

void MusclePlugin::Init()
{
	//state initialization
    x[0] = 0.0; // start at i=0.0, w_g=0.0
    x[1] = 0.0;
    _motor.voltage = 0.0;
    _spindle.angVel = 0;
	// insertion = math::Vector3(1, 1, 1);//armPose.pos;
	// fixation = math::Vector3(2, 2, 2);//handPose.pos;
    
}


void MusclePlugin::OnUpdate()
{
	
	

	//compute the step time
	common::Time currTime = this->model->GetWorld()->GetSimTime();
    common::Time stepTime = currTime - this->prevUpdateTime;
 	this->prevUpdateTime = currTime;


	// calculate elastic force 
	_see.length = (fixation - insertion).GetLength();
	elasticForce = tendon.ElasticElementModel(_see.lengthRest, 
		_see.length, _see.stiffness, _spindle.angVel, _spindle.radius);
	
	// calculate motor force

	// calculate the approximation of gear's efficiency
	_gear.appEfficiency = actuator.EfficiencyApproximation(); 

	// do 1 step of integration of DiffModel() at current time
	actuator.stepper.do_step(IActuator::DiffModel, x, currTime.Double(), stepTime.Double());
	_motor.current = x[0];
 	_spindle.angVel = x[1];

 	actuatorForce = tendon.ElectricMotorModel(_motor.current, _motor.torqueConst,
 		_spindle.radius);
    

    // calculate joint force (elastic+actuator)
    math::Vector3 force = tendon.CalculateForce(elasticForce, actuatorForce, 
    	insertion, fixation);
   
    if ((int) currTime.Double() % 100000 == 0)
    {
	    gzdbg << "applying force: " 
			  << force
			  << "\n"
			  << "with elasticForce: "
			  << elasticForce
			  << " and motorForce: "
			  << actuatorForce
			  << " and length: "
			  << _see.length
			  << "\n";	
    }

    // for (auto x: fixation)
    // {
    // 	x++;
    // } 

}