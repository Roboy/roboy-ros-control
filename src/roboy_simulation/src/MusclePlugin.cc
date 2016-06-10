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

// struct tendonType {
//   math::Vector3 MidPoint[linkNumber-1];
//   math::Vector3 Vector[linkNumber-1];//might need it to calculate length
//   math::Vector3 Orientation[linkNumber-1];
//   double Pitch[linkNumber-1];
//   double Roll[linkNumber-1];
// } ;

// const int linkNumber = 3; //later read this number from some table or .txt files
transport::NodePtr node;
transport::PublisherPtr visPub;

msgs::Visual visualMsg[linkNumber-1];

math::Vector3 linkRelVec[linkNumber];

float elasticForce;
float actuatorForce;

MusclePlugin::MusclePlugin()
{

}

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
							 const float _speed, const float _spindleRadius, const double _time)
{
    // float realTimeUpdateRate=1000;
    float windingLength = _spindleRadius*_speed*_time;
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
	const math::Vector3 &_tendonOrien)
{
	// math::Vector3 diff = _fixationP - _instertionP;
    
	/*    float tendonForce;
    
    if (_elasticForce+_motorForce>=0)
    {
        tendonForce=_elasticForce+_motorForce;
    }
    else
    {
        tendonForce=0;
    }*/
    
	return _tendonOrien*(_elasticForce+_motorForce);
    
}

 void ITendon::GetTendonInfo(math::Vector3 _viaPointPose[],tendonType *tendon_p)//try later with pointer
{

	for (int i = 0; i<linkNumber-1; i++)
	{
			tendon_p->MidPoint[i] = (_viaPointPose[i] +_viaPointPose[i+1])/2;
			tendon_p->Vector[i] = _viaPointPose[i]-_viaPointPose[i+1];
			tendon_p->Orientation[i] = tendon_p->Vector[i]/tendon_p->Vector[i].GetLength();
			tendon_p->Pitch[i] = static_cast<double>(atan(tendon_p->Orientation[i][0]/tendon_p->Orientation[i][2]));
			tendon_p->Roll[i]= static_cast<double>(-acos(sqrt((pow(tendon_p->Orientation[i][0],2)+pow(tendon_p->Orientation[i][2],2)))));
	}
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
			!motorElement->HasElement("bemf_constant") ||
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


	std::ifstream myfile ("TendonInfo.txt");
  	std::string linkNameTmp[linkNumber];
	if (myfile.is_open())
  	{
    	int i=0;
    	for ( std::string element; getline (myfile,element,' ');)
    	{

      		char ch;
      		double x,y,z;
      		std::vector<int> array;
      		std::stringstream ss(element);

    		if (ss>>x>>ch>>y>>ch>>z) //If input to string successfully; I know it is rundundant some expert optimize this part plz
    		{
    			linkRelVec[i].Set(x,y,z);
    			i++;
    		}else linkNameTmp[i]=element;//if not double then element is the name

    	}
    	std::cout << "Link relative vectors: \n" 
    		<< linkRelVec[0] << "\t" 
    		<< linkRelVec[1] << "\t" 
    		<< linkRelVec[2]<<'\n';

    	myfile.close();
  	}
  	else std::cout << "Unable to open file";

	for(auto linkName: linkNameTmp)
	{
		physics::LinkPtr link = this->model->GetLink(linkName);
    	if (!link)
    	{
	        gzwarn << "Invalid SDF: model link " << linkName << " does not "
	               << "exist!" << std::endl;
	        continue;
    	}
    	this->links.push_back(link);
	}


	this->connection = event::Events::ConnectWorldUpdateBegin(
         			 boost::bind(&MusclePlugin::OnUpdate, this));

	//create a cylinder as visual

    node = transport::NodePtr(new transport::Node());
    node->Init(_parent->GetWorld()->GetName());
    visPub = node->Advertise<msgs::Visual>("~/visual", 10);

    // Set the visual's name. This should be unique.

    for (int i=0; i < linkNumber - 1; i++)
    {
    	std::stringstream ss;
		ss << "CYLINDER_VISUAL__" << i;
		std::string visualName = ss.str();
    	visualMsg[i].set_name(visualName);

    	//Set the visual's parent. This visual will be attached to the parent
    	//visualMsg.set_parent_name(_parent->GetScopedName());
    	visualMsg[i].set_parent_name(links[i]->GetName());
    	msgs::Geometry *geomMsg = visualMsg[i].mutable_geometry();

    	//std::cout << _parent->GetScopedName();
    	// Create a cylinder

    	geomMsg->set_type(msgs::Geometry::CYLINDER);
    	geomMsg->mutable_cylinder()->set_radius(.001);
    	visualMsg[i].set_cast_shadows(false);
	}
}

void MusclePlugin::Init()
{
	//state initialization
    x[0] = 0.0; // start at i=0.0, w_g=0.0
    x[1] = 0.0;
    _motor.voltage = 0.0;
    _spindle.angVel = 0;

    // get bounding box of the link 
    // gzmsg << "CoG 1 : " << links[0]->GetWorldCoGPose() << "\n";
    // gzmsg << "CoG 2 : " << links[1]->GetWorldCoGPose() << "\n";
    // rendering::WireBox::WireBox	(this->model, links[0]-> GetBoundingBox());
	// links[0]-> ShowBoundingBox();
	// links[1]-> ShowBoundingBox();

	// insertion = math::Vector3(1, 1, 1);//armPose.pos;
	// fixation = math::Vector3(2, 2, 2);//handPose.pos;
	
	

    
}


void MusclePlugin::OnUpdate()
{
	
	//compute the step time
	common::Time currTime = this->model->GetWorld()->GetSimTime();
    common::Time stepTime = currTime - this->prevUpdateTime;
 	this->prevUpdateTime = currTime;
	
	if ( fmod(currTime.Double(),0.2) == 0 )
    {
		std::cout << "Voltage:";
		std::cin >> _motor.voltage;
	}

	math::Vector3 force;
	math::Vector3 viaPointPos[linkNumber];
	tendonType newTendon;

	// get the position and orientation of the links
	for (int i = 0; i<linkNumber; i++)
	{
		const math::Pose linkPose= links[i]->GetWorldCoGPose();

		// absolute position + relative position=actual position of each via point
		viaPointPos[i]=linkPose.pos+linkPose.rot.RotateVector(linkRelVec[i]);
	}

	tendon.GetTendonInfo(viaPointPos, &newTendon);	

	// extract info from linkPose
	_see.length = newTendon.Vector[0].GetLength() + newTendon.Vector[1].GetLength();


	// calculate elastic force 
	elasticForce = 0;//tendon.ElasticElementModel(_see.lengthRest, 
		// _see.length, _see.stiffness, _spindle.angVel,
		// _spindle.radius, stepTime.Double());
	
	
	// calculate motor force
	// calculate the approximation of gear's efficiency
	_gear.appEfficiency = actuator.EfficiencyApproximation(); 

	// do 1 step of integration of DiffModel() at current time
	actuator.stepper.do_step(IActuator::DiffModel, x, currTime.Double(), 
		stepTime.Double());
	
	 // gzdbg << "electric current: " 
		// 	  << x[0]
		// 	  << "\t"
		// 	  << "speed: "
		// 	  << x[1]
		// 	  << "\n";

	_motor.current = x[0];
 	_spindle.angVel = x[1];

 	actuatorForce = tendon.ElectricMotorModel(_motor.current, _motor.torqueConst,
 		_spindle.radius);
    
   
    // calculate general force (elastic+actuator)
 	for (int i = 0; i < linkNumber - 1; i++)
 	{

 		//remove elastic force, this is to be discussed
		force = tendon.CalculateForce(elasticForce, actuatorForce, newTendon.Orientation[i]);

	    this->links[i]->AddForceAtWorldPosition(-force, viaPointPos[i]);
	    this->links[i+1]->AddForceAtWorldPosition(force,viaPointPos[i+1]);

	    //update position and orientation, somehow float not accepted, must use double instead. for pricision maybe
		// TODO: the following would not compile under ros jade and gazebo 5.3, check whats wrong
//	    msgs::Set(visualMsg[i].mutable_pose(),ignition::math::Pose3d(
//	    		ignition::math::Vector3d(static_cast<double>(newTendon.MidPoint[i][0]),
//	    			static_cast<double>(newTendon.MidPoint[i][1]),
//	    			static_cast<double>(newTendon.MidPoint[i][2])),
//	    		ignition::math::Vector3d(newTendon.Roll[i],newTendon.Pitch[i],0)));
	    msgs::Geometry *geomMsg = visualMsg[i].mutable_geometry();

	    //update length
		geomMsg->mutable_cylinder()->set_length(newTendon.Vector[i].GetLength());
		visPub->Publish(visualMsg[i]);//show updated visual

	}
    

}