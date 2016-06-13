#include "TendonPlugin.hh"

using namespace gazebo;


//Register plugin with this simulator
GZ_REGISTER_MODEL_PLUGIN(TendonPlugin);

MotorProperties motor;
SEEProperties SEE;
int timeCounter;

TendonPlugin::TendonPlugin()
{

}

float TendonPlugin::DotProduct(const math::Vector3 &_v1, const math::Vector3 &_v2)
{
	return _v1.x*_v2.x + _v1.y*_v2.y + _v1.z*_v2.z;
}


float TendonPlugin::Angle(const math::Vector3 &_v1, const math::Vector3 &_v2)
{
	return acos(_v1.Dot(_v2)/_v1.GetLength()*_v2.GetLength());
}


float TendonPlugin::ElectricMotorModel(const float _current,  const float _torqueConstant, 
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


float TendonPlugin::ElasticElementModel(const float _length0, const float _length, float _stiffness[],
							 const float _speed, const float _spindleRadius)
{
    float realTimeUpdateRate=1000;
    float windingLength=_spindleRadius*_speed/realTimeUpdateRate*timeCounter;
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
        elasticForce=displacement*_stiffness[0];
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


math::Vector3 TendonPlugin::CalculateForce(float _elasticForce, float _motorForce, 
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

void TendonPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	
	//Store pointer to the model
	this->model = _parent;



	//Get the parameters from SDF
	gzmsg << "Reading values from SDF" << std::endl;
	if (_sdf->HasElement("motor"))
	{	
		sdf::ElementPtr motorElement = _sdf->GetElement("motor");

	/*		if (!motorElement->HasElement("electric_current") || 
			!motorElement->HasElement("torque_constant"))	
		{
			gzwarn << "Invalid SDF: Missing required elements for motor model";
		}*/
		
   		if (!motorElement->HasElement("torque_constant"))
		{
			gzwarn << "Invalid SDF: Missing required elements for motor model";
		}
        
	/*		if (motorElement->HasElement("electric_current"))
		{
			float current;	
			motor.eCurrent = motorElement->Get<float>("electric_current");	
		}*/

	/*		if (motorElement->HasElement("torque_constant"))
		{
			//float t_const;	
			motor.torqueConst = motorElement->Get<float>("torque_constant");	
		}*/

			//float t_const;	
			motor.torqueConst = motorElement->Get<float>("torque_constant");
            
	/*			//float speed;	
			motor.speed = motorElement->Get<float>("speed");*/
	}

	if(_sdf->HasElement("spindle"))
	{
		sdf::ElementPtr spindleElement = _sdf->GetElement("spindle");
		if (!spindleElement->HasElement("radius"))	
		{
			gzwarn << "Invalid SDF: Missing required elements for spindle";
		}
			//float spindle_r;	
			motor.spindleRadius = spindleElement->Get<float>("radius");	
	}
		

	if (_sdf->HasElement("SEE"))
	{	
		
		sdf::ElementPtr elasticElement = _sdf->GetElement("SEE");

	/*		if (!elasticElement->HasElement("stiffness") || 
			!elasticElement->HasElement("length") ||
			 !elasticElement->HasElement("length0"))
		{
			gzwarn << "Invalid SDF: Missing required elements for series elastic element";
		}*/
        
		if (!elasticElement->HasElement("stiffness") ||
			 !elasticElement->HasElement("length0"))
		{
			gzwarn << "Invalid SDF: Missing required elements for series elastic element";
		}

	/*		if (elasticElement->HasElement("stiffness"))
		{
			sdf::ElementPtr stiffnessElement = elasticElement->GetElement("stiffness");
			//float stiffness[4];

			std::string stiffnessString = elasticElement->Get<std::string>("stiffness");

			//get floats from string
			std::istringstream ss(stiffnessString);
			std::copy(std::istream_iterator <float> (ss),
				std::istream_iterator <float>(),
				SEE.stiffness);

			//for(int i=0; i<4; i++)
			//	gzmsg << "stiffnessElement: " << stiffness[i] << std::endl;
				
		}*/
            //stiffness
			sdf::ElementPtr stiffnessElement = elasticElement->GetElement("stiffness");
			//float stiffness[4];

			std::string stiffnessString = elasticElement->Get<std::string>("stiffness");

			//get floats from string
			std::istringstream ss(stiffnessString);
			std::copy(std::istream_iterator <float> (ss),
				std::istream_iterator <float>(),
				SEE.stiffness);

	/*		if (elasticElement->HasElement("length"))
		{
			float length;	
			SEE.length = elasticElement->Get<float>("length");	
		}*/


		if (elasticElement->HasElement("length0"))
		{
			//float length0;	
			SEE.lengthRest = elasticElement->Get<float>("length0");	
		}



	/*		std::vector<std::string> linkNames = {"upper_arm", "hand"};
		for(auto linkName: linkNames)
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
         			 boost::bind(&TendonPlugin::OnUpdate, this));*/

	}

		std::vector<std::string> linkNames = {"upper_arm", "hand"};
		for(auto linkName: linkNames)
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
         			 boost::bind(&TendonPlugin::OnUpdate, this));

} 

void TendonPlugin::Init()
{

}


void TendonPlugin::OnUpdate()
{
	const math::Pose armPose = links[0]->GetWorldCoGPose();
	const math::Pose handPose = links[1]->GetWorldCoGPose();

	math::Vector3 insertion = armPose.pos;//math::Vector3(armPose[0], armPose[1], armPose[2]);
	math::Vector3 fixation = handPose.pos;//math::Vector3(handPose[0], handPose[1], handPose[2]);

	//motor force
//	//electric current input
//	if(timeCounter%5000 == 0)
//	{
//		float cur;
//		std::cout << "Motor's current:";
//		std::cin >> motor.eCurrent;
//        std::cout << "Motor's speed:";
//		std::cin >> motor.speed;
//	}

	//elastic force 
	SEE.length = (fixation - insertion).GetLength();
	float elasticForce = ElasticElementModel(SEE.lengthRest, SEE.length, SEE.stiffness, motor.speed, motor.spindleRadius);
	

	float motorForce = ElectricMotorModel(motor.eCurrent, motor.torqueConst, motor.spindleRadius);

    math::Vector3 force = CalculateForce(elasticForce, motorForce, insertion, fixation);
    if(timeCounter%200 == 0)
    {
    gzdbg << "applying force: " 
		  << force
		  << "\n"
		  << "with elasticForce: "
		  << elasticForce
		  << " and motorForce: "
		  << motorForce
		  << " and length: "
		  << SEE.length
		  << "\n";	
    }
	

	this->links[0]->AddForceAtWorldPosition(force, insertion);
	this->links[1]->AddForceAtWorldPosition(-force, fixation);
	//gazebo::common::Time::MSleep(120);
	// this->links[0]->AddForceAtWorldPosition(-force, insertion);
	// this->links[1]->AddForceAtWorldPosition(force, fixation);
	// gazebo::common::Time::MSleep(120);
    
    ++timeCounter;
}


	/*
	void TendonPlugin::MakeTendon(const std::string &_name, math::Pose &_pose, 
								double _mass, double _radius, double _lenght)
	{
		std::ostringstream newModelStr;

		newModelStr << "<sdf version='" << SDF_VERSION << "'>"
			"<model name='" << _name << "'>"
			"<pose>" << _pose << "</pose>"
			"<link name='link'>"
			   "<gravity>true</gravity>"
			   "<pose>{pose}</pose>"
			    "<visual name='visual'>"
			      "<geometry>"
			        "<cylinder>"
			          "<length>" << _length << "</length>"
			          "<radius>" << _radius << "</radius>"
			        "</cylinder>"
			      "</geometry>"
			    "</visual>"
			  "</link>"
			"</model"
			"</sdf>"


		this->world->InsertModelString(newModelStr.str());
	}
	*/


