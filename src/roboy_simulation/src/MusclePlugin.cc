#include <CommunicationData.h>
#include "MusclePlugin.hh"

namespace roboy_simulation {

	double PIDcontroller::calc_output(double cmd, double pos, double timePeriod) {
		float pterm, dterm, result, err, ffterm;
		if (cmd >= spNegMax && cmd <= spPosMax) {
			err = pos - cmd;
			if ((err > params.pidParameters.deadBand) || (err < -1 * params.pidParameters.deadBand)) {
				pterm = params.pidParameters.pgain * err;
				if ((pterm < outputPosMax) || (pterm > outputNegMax)) //if the proportional term is not maxed
				{
					integral += (params.pidParameters.igain * err * (timePeriod)); //add to the integral
					if (integral > params.pidParameters.IntegralPosMax)
						integral = params.pidParameters.IntegralPosMax;
					else if (integral < params.pidParameters.IntegralNegMax)
						integral = params.pidParameters.IntegralNegMax;
				}

				dterm = ((err - lastError) / timePeriod) * params.pidParameters.dgain;

				ffterm = params.pidParameters.forwardGain * cmd;
				result = ffterm + pterm + integral + dterm;
				if (result < outputNegMax)
					result = outputNegMax;
				else if (result > outputPosMax)
					result = outputPosMax;
			}
			else
				result = integral;
			lastError = err;
		} else {
			result = 0;
		}
		return result;
	}

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

	void ITendon::GetTendonInfo(vector<math::Vector3> &viaPointPos, tendonType *tendon_p)//try later with pointer
	{
		for (int i = 0; i < viaPointPos.size() - 1; i++) {
			tendon_p->MidPoint.push_back((viaPointPos[i] + viaPointPos[i + 1]) / 2);
			tendon_p->Vector.push_back(viaPointPos[i] - viaPointPos[i + 1]);
			tendon_p->Orientation.push_back(tendon_p->Vector[i] / tendon_p->Vector[i].GetLength());
			tendon_p->Pitch.push_back(atan(tendon_p->Orientation[i][0] / tendon_p->Orientation[i][2]));
			tendon_p->Roll.push_back(
					-acos(sqrt((pow(tendon_p->Orientation[i][0], 2) + pow(tendon_p->Orientation[i][2], 2)))));
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

	void MusclePlugin::Init(MyoMuscleInfo &myoMuscle) {
		//state initialization
		x[0] = 0.0; // start at i=0.0, w_g=0.0
		x[1] = 0.0;
		actuator.motor.voltage = 0.0;
		actuator.spindle.angVel = 0;

		joint = myoMuscle.joint;
		viaPoints = myoMuscle.viaPoints;
		actuator.motor = myoMuscle.motor;
		actuator.gear = myoMuscle.gear;
		actuator.spindle = myoMuscle.spindle;
		tendon.see = myoMuscle.see;


		pid.params.pidParameters.pgain = 1000;
		pid.params.pidParameters.igain = 0;
		pid.params.pidParameters.dgain = 0;
	}


	void MusclePlugin::Update(ros::Time &time, ros::Duration &period,
							  vector<math::Vector3> &viaPointInGobalFrame,
							  vector<math::Vector3> &force) {
		// TODO: calculate PID result
//		pid.calc_output(cmd,pos,period);
		actuator.motor.voltage = cmd;

		tendonType newTendon;

		// get the position and orientation of the links
		for (auto viaPoint:viaPoints) {
			// absolute position + relative position=actual position of each via point
			for (uint i = 0; i < viaPoint.second.size(); i++)
				viaPointInGobalFrame.push_back(
						linkPose[viaPoint.first].pos + linkPose[joint[i]].rot.RotateVector(viaPoint.second[i]));
		}

		tendon.GetTendonInfo(viaPointInGobalFrame, &newTendon);

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
													   actuator.motor.BEMFConst * actuator.gear.ratio * x[1] +
													   actuator.motor.voltage);
			dxdt[1] = actuator.motor.torqueConst * x[0] / (actuator.gear.ratio * totalIM) -
					  actuator.spindle.radius * actuator.elasticForce /
					  (actuator.gear.ratio * actuator.gear.ratio * totalIM * actuator.gear.appEfficiency);
		}, x, time.sec, period.sec);

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
		for (auto viaPoint:viaPoints) {
			//remove elastic force, this is to be discussed
			for (uint i = 0; i < viaPoint.second.size(); i++)
				force.push_back(tendon.CalculateForce(actuator.elasticForce, actuatorForce, newTendon.Orientation[i]));
//			this->links[i]->AddForceAtWorldPosition(-force, viaPointPos[i]);
//			this->links[i + 1]->AddForceAtWorldPosition(force, viaPointPos[i + 1]);

			//update position and orientation, somehow double not accepted, must use double instead. for pricision maybe
			// TODO: the following would not compile under ros jade and gazebo 5.3, check whats wrong
//	    msgs::Set(visualMsg[i].mutable_pose(),ignition::math::Pose3d(
//	    		ignition::math::Vector3d(newTendon.MidPoint[i][0], newTendon.MidPoint[i][1], newTendon.MidPoint[i][2]),
//	    		ignition::math::Vector3d(newTendon.Roll[i],newTendon.Pitch[i],0)));
//			msgs::Geometry *geomMsg = visualMsg[i].mutable_geometry();

//			//update length
//			geomMsg->mutable_cylinder()->set_length(newTendon.Vector[i].GetLength());
//			visPub->Publish(visualMsg[i]);//show updated visual

		}
	}
}
// make it a plugin loadable via pluginlib
PLUGINLIB_EXPORT_CLASS(roboy_simulation::MusclePlugin, roboy_simulation::MusclePlugin)