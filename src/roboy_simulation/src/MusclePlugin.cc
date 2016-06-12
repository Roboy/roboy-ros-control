#include "MusclePlugin.hh"

using namespace gazebo;

double ITendon::ElectricMotorModel(const double _current, const double _torqueConstant,
								   const double _spindleRadius) {
	double motorForce;

	if (_current >= 0) {
		motorForce = _current * _torqueConstant / _spindleRadius;
	}
	else {
		motorForce = 0;
	}

	return motorForce;
}


double ITendon::ElasticElementModel(const double _length0, const double _length, double _stiffness,
									const double _speed, const double _spindleRadius, const double _time) {
	// double realTimeUpdateRate=1000;
	double windingLength = _spindleRadius * _speed * _time;
	double displacement;
	displacement = windingLength + _length - _length0;

	// gzdbg << "displacement: "
	// 	  << displacement
	// 	  << "\n"
	//          << "windingLength: "
	// 	  << windingLength
	// 	  << "\n";

	double elasticForce;

	if (displacement >= 0) {
		elasticForce = displacement * _stiffness;
	}
	else {
		elasticForce = 0;
	}

	//return _stiffness[0] + (displacement*_stiffness[1]) + (displacement*displacement*_stiffness[2]) +
	//			(displacement*displacement*displacement*_stiffness[3]) ;
	//return displacement*_stiffness[0];
	return elasticForce;

}

math::Vector3 ITendon::CalculateForce(double _elasticForce, double _motorForce,
									  const math::Vector3 &_tendonOrien) {
	// math::Vector3 diff = _fixationP - _instertionP;

	/*    double tendonForce;

    if (_elasticForce+_motorForce>=0)
    {
        tendonForce=_elasticForce+_motorForce;
    }
    else
    {
        tendonForce=0;
    }*/

	return _tendonOrien * (_elasticForce + _motorForce);

}

void ITendon::GetTendonInfo(math::Vector3 _viaPointPose[], tendonType *tendon_p)//try later with pointer
{

	for (int i = 0; i < linkNumber - 1; i++) {
		tendon_p->MidPoint[i] = (_viaPointPose[i] + _viaPointPose[i + 1]) / 2;
		tendon_p->Vector[i] = _viaPointPose[i] - _viaPointPose[i + 1];
		tendon_p->Orientation[i] = tendon_p->Vector[i] / tendon_p->Vector[i].GetLength();
		tendon_p->Pitch[i] = static_cast<double>(atan(tendon_p->Orientation[i][0] / tendon_p->Orientation[i][2]));
		tendon_p->Roll[i] = static_cast<double>(-acos(
				sqrt((pow(tendon_p->Orientation[i][0], 2) + pow(tendon_p->Orientation[i][2], 2)))));
	}
}

double ITendon::DotProduct(const math::Vector3 &_v1, const math::Vector3 &_v2) {
	return _v1.x * _v2.x + _v1.y * _v2.y + _v1.z * _v2.z;
}


double ITendon::Angle(const math::Vector3 &_v1, const math::Vector3 &_v2) {
	return acos(_v1.Dot(_v2) / _v1.GetLength() * _v2.GetLength());
}

double IActuator::EfficiencyApproximation() {
	double param1 = 0.1; // defines steepness of the approximation
	double param2 = 0; // defines zero crossing of the approximation
	return gear.efficiency + (1 / gear.efficiency - gear.efficiency) *
							 (0.5 * (tanh(-param1 * spindle.angVel * motor.current - param2) + 1));
}

MusclePlugin::MusclePlugin() {
	x.resize(2);
}

void MusclePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
	//Store pointer to the model
	this->model = _parent;

	//Get the parameters from SDF
	gzmsg << "Reading values from SDF" << std::endl;

	if (_sdf->HasElement("motor")) {
		sdf::ElementPtr motorElement = _sdf->GetElement("motor");

		if (//!motorElement->HasElement("electric_current") || 
				!motorElement->HasElement("torque_constant") ||
				!motorElement->HasElement("bemf_constant") ||
				!motorElement->HasElement("inductance") ||
				!motorElement->HasElement("resistance") ||
				!motorElement->HasElement("inertiaMoment")) {
			gzwarn << "Invalid SDF: Missing required elements for motor model";
		}

		if (motorElement->HasElement("torque_constant")) {
			actuator.motor.torqueConst = motorElement->Get<double>("torque_constant");
			gzmsg << "torque_constant " << actuator.motor.torqueConst << "\n";
		}

		if (motorElement->HasElement("bemf_constant")) {
			actuator.motor.BEMFConst = motorElement->Get<double>("bemf_constant");
			gzmsg << "bemf_constant " << actuator.motor.BEMFConst << "\n";
		}

		if (motorElement->HasElement("inductance")) {
			actuator.motor.inductance = motorElement->Get<double>("inductance");
			gzmsg << "inductance " << actuator.motor.inductance << "\n";
		}

		if (motorElement->HasElement("resistance")) {
			actuator.motor.resistance = motorElement->Get<double>("resistance");
			gzmsg << "resistance " << actuator.motor.resistance << "\n";
		}

		if (motorElement->HasElement("inertiaMoment")) {
			actuator.motor.inertiaMoment = motorElement->Get<double>("inertiaMoment");
			gzmsg << "inertia " << actuator.motor.inertiaMoment << "\n";
		}
	}

	if (_sdf->HasElement("gear")) {
		sdf::ElementPtr gearElement = _sdf->GetElement("gear");

		if (!gearElement->HasElement("ratio") ||
			!gearElement->HasElement("efficiency") ||
			!gearElement->HasElement("inertiaMoment")) {
			gzwarn << "Invalid SDF: Missing required elements for gear model";
		}

		if (gearElement->HasElement("ratio")) {
			actuator.gear.ratio = gearElement->Get<double>("ratio");
			gzmsg << "ratio " << actuator.gear.ratio << "\n";
		}

		if (gearElement->HasElement("efficiency")) {
			actuator.gear.efficiency = gearElement->Get<double>("efficiency");
			gzmsg << "efficiency " << actuator.gear.efficiency << "\n";
		}

		if (gearElement->HasElement("inertiaMoment")) {
			actuator.gear.inertiaMoment = gearElement->Get<double>("inertiaMoment");
			gzmsg << "inertia " << actuator.gear.inertiaMoment << "\n";
		}
	}


	if (_sdf->HasElement("spindle")) {
		sdf::ElementPtr spindleElement = _sdf->GetElement("spindle");

		if (!spindleElement->HasElement("radius")) {
			gzwarn << "Invalid SDF: Missing required elements for spindle model";
		}

		if (spindleElement->HasElement("radius")) {
			actuator.spindle.radius = spindleElement->Get<double>("radius");
			gzmsg << "radius " << actuator.spindle.radius << "\n";
		}
	}

	if (_sdf->HasElement("SEE")) {

		sdf::ElementPtr elasticElement = _sdf->GetElement("SEE");


		if (!elasticElement->HasElement("stiffness") ||
			!elasticElement->HasElement("length0")) {
			gzwarn << "Invalid SDF: Missing required elements for series elastic element";
		}

		//stiffness
		// sdf::ElementPtr stiffnessElement = elasticElement->GetElement("stiffness");
		//double stiffness[4];

		if (elasticElement->HasElement("stiffness")) {
			tendon.see.stiffness = elasticElement->Get<double>("stiffness");
		}


		//get floats from string
		// std::istringstream ss(stiffnessString);
		// std::copy(std::istream_iterator <double> (ss),
		// 	std::istream_iterator <double>(),
		// 	SEE.stiffness);

		if (elasticElement->HasElement("length0")) {
			tendon.see.lengthRest = elasticElement->Get<double>("length0");
		}

	}

	vector<string> linkNameTmp(3);
	linkNameTmp[0] = "upper_arm";
	linkNameTmp[1] = "hand";
	linkNameTmp[2] = "hand";

    linkRelVec[0] = math::Vector3(0.0,-0.1,0.0);
	linkRelVec[1] = math::Vector3(0.0,-0.11,0.0);
	linkRelVec[2] = math::Vector3(0.0,0.0,0.0);

	for (auto linkName: linkNameTmp) {
		physics::LinkPtr link = this->model->GetLink(linkName);
		if (!link) {
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

//	for (int i = 0; i < linkNumber - 1; i++) {
//		std::stringstream ss;
//		ss << "CYLINDER_VISUAL__" << i;
//		std::string visualName = ss.str();
//		visualMsg[i].set_name(visualName);
//
//		//Set the visual's parent. This visual will be attached to the parent
//		//visualMsg.set_parent_name(_parent->GetScopedName());
//		visualMsg[i].set_parent_name(links[i]->GetName());
//		msgs::Geometry *geomMsg = visualMsg[i].mutable_geometry();
//
//		//std::cout << _parent->GetScopedName();
//		// Create a cylinder
//
//		geomMsg->set_type(msgs::Geometry::CYLINDER);
//		geomMsg->mutable_cylinder()->set_radius(.001);
//		visualMsg[i].set_cast_shadows(false);
//	}
}

void MusclePlugin::Init() {
	//state initialization
	x[0] = 0.0; // start at i=0.0, w_g=0.0
	x[1] = 0.0;
	actuator.motor.voltage = 0.0;
	actuator.spindle.angVel = 0;

	// get bounding box of the link
	// gzmsg << "CoG 1 : " << links[0]->GetWorldCoGPose() << "\n";
	// gzmsg << "CoG 2 : " << links[1]->GetWorldCoGPose() << "\n";
	// rendering::WireBox::WireBox	(this->model, links[0]-> GetBoundingBox());
	// links[0]-> ShowBoundingBox();
	// links[1]-> ShowBoundingBox();

	// insertion = math::Vector3(1, 1, 1);//armPose.pos;
	// fixation = math::Vector3(2, 2, 2);//handPose.pos;
}


void MusclePlugin::OnUpdate() {

	//compute the step time
	common::Time currTime = this->model->GetWorld()->GetSimTime();
	common::Time stepTime = currTime - this->prevUpdateTime;
	this->prevUpdateTime = currTime;

	if (fmod(currTime.Double(), 0.2) == 0) {
		std::cout << "Voltage:";
		std::cin >> actuator.motor.voltage;
	}

	math::Vector3 force;
	math::Vector3 viaPointPos[linkNumber];
	tendonType newTendon;

	// get the position and orientation of the links
	for (int i = 0; i < linkNumber; i++) {
		const math::Pose linkPose = links[i]->GetWorldCoGPose();

		// absolute position + relative position=actual position of each via point
		viaPointPos[i] = linkPose.pos + linkPose.rot.RotateVector(linkRelVec[i]);
	}

	tendon.GetTendonInfo(viaPointPos, &newTendon);

	// extract info from linkPose
	tendon.see.length = newTendon.Vector[0].GetLength() + newTendon.Vector[1].GetLength();


	// calculate elastic force
	actuator.elasticForce = 0;//tendon.ElasticElementModel(_see.lengthRest,
	// _see.length, _see.stiffness, spindle.angVel,
	// spindle.radius, stepTime.Double());


	// calculate motor force
	// calculate the approximation of gear's efficiency
	actuator.gear.appEfficiency = actuator.EfficiencyApproximation();

	// do 1 step of integration of DiffModel() at current time
	actuator.stepper.do_step([this](const IActuator::state_type &x, IActuator::state_type &dxdt, const double) {
		// This lambda function describes the differential model for the simulations of dynamics
		// of a DC motor, a spindle, and a gear box
		// x[0] - motor electric current
		// x[1] - spindle angular velocity
		double totalIM = actuator.motor.inertiaMoment + actuator.gear.inertiaMoment; // total moment of inertia
		dxdt[0] = 1 / actuator.motor.inductance * (-actuator.motor.resistance * x[0] -
				actuator.motor.BEMFConst * actuator.gear.ratio * x[1] + actuator.motor.voltage);
		dxdt[1] = actuator.motor.torqueConst * x[0] / (actuator.gear.ratio * totalIM) -
				actuator.spindle.radius * actuator.elasticForce /
						(actuator.gear.ratio * actuator.gear.ratio * totalIM * actuator.gear.appEfficiency);
	}, x, currTime.Double(), stepTime.Double());

	// gzdbg << "electric current: "
	// 	  << x[0]
	// 	  << "\t"
	// 	  << "speed: "
	// 	  << x[1]
	// 	  << "\n";

	actuator.motor.current = x[0];
	actuator.spindle.angVel = x[1];

	actuatorForce = tendon.ElectricMotorModel(actuator.motor.current, actuator.motor.torqueConst,
											  actuator.spindle.radius);

	// calculate general force (elastic+actuator)
	for (int i = 0; i < linkNumber - 1; i++) {

		//remove elastic force, this is to be discussed
		force = tendon.CalculateForce(actuator.elasticForce, actuatorForce, newTendon.Orientation[i]);

		this->links[i]->AddForceAtWorldPosition(-force, viaPointPos[i]);
		this->links[i + 1]->AddForceAtWorldPosition(force, viaPointPos[i + 1]);

		//update position and orientation, somehow double not accepted, must use double instead. for pricision maybe
		// TODO: the following would not compile under ros jade and gazebo 5.3, check whats wrong
//	    msgs::Set(visualMsg[i].mutable_pose(),ignition::math::Pose3d(
//	    		ignition::math::Vector3d(newTendon.MidPoint[i][0], newTendon.MidPoint[i][1], newTendon.MidPoint[i][2]),
//	    		ignition::math::Vector3d(newTendon.Roll[i],newTendon.Pitch[i],0)));
		msgs::Geometry *geomMsg = visualMsg[i].mutable_geometry();

		//update length
		geomMsg->mutable_cylinder()->set_length(newTendon.Vector[i].GetLength());
		visPub->Publish(visualMsg[i]);//show updated visual

	}
}

//Register plugin with this simulator
GZ_REGISTER_MODEL_PLUGIN(MusclePlugin);