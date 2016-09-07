#include "MusclePlugin.hpp"

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

	MusclePlugin::MusclePlugin() {
		x.resize(2);
	}

	void MusclePlugin::Init(MyoMuscleInfo &myoMuscle) {
		//state initialization
		x[0] = 0.0; // start at i=0.0, w_g=0.0
		x[1] = 0.0;
		actuator.motor.voltage = 0.0;
		actuator.spindle.angVel = 0;

		viaPoints = myoMuscle.viaPoints;
		actuator.motor = myoMuscle.motor;
		actuator.gear = myoMuscle.gear;
		actuator.spindle = myoMuscle.spindle;
		tendon.see = myoMuscle.see;
        name = myoMuscle.name;
		tendon.see.expansion = 0.0;
		tendon.see.force = 0.0;

		pid.params.pidParameters.pgain = 1000;
		pid.params.pidParameters.igain = 0;
		pid.params.pidParameters.dgain = 0;
	}

	void MusclePlugin::Update( ros::Time &time, ros::Duration &period ) {

		// TODO: calculate PID result
//		pid.calc_output(cmd,pos,period);
		actuator.motor.voltage = cmd;

        for(int i = 0; i < viaPoints.size(); i++){
            // absolute position + relative position=actual position of each via point
            math::Pose linkPose = viaPoints[i].link->GetWorldPose();
            viaPoints[i].linkPosition = linkPose.pos;
            viaPoints[i].linkRotation = linkPose.rot;
            viaPoints[i].globalCoordinates = viaPoints[i].linkPosition + viaPoints[i].linkRotation.RotateVector(viaPoints[i].localCoordinates);
        }

        //update force points and calculate muscle length
        for(int i = 0; i < viaPoints.size(); i++)
        {
            viaPoints[i].UpdateForcePoints();
            tendon.muscleLength += viaPoints[i].previousSegmentLength;
        };

		if (tendon.firstUpdate)
        {
            tendon.initialTendonLength = tendon.muscleLength;
            tendon.tendonLength = tendon.muscleLength;
            tendon.firstUpdate = false;
        }

        //calculate elastic force
        tendon.see.length = tendon.muscleLength - tendon.tendonLength;
        tendon.ElasticElementModel(tendon.see, tendon.see.length);
        actuator.elasticForce = tendon.see.force;
        //set elastic force zero to compare with old plugin functionality
        actuator.elasticForce = 0;

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
		}, x, time.nsec*1e-6, period.nsec*1e-6);

		actuator.motor.current = x[0];
		actuator.spindle.angVel = x[1];

		//calculate motor force
		actuatorForce = tendon.ElectricMotorModel(actuator.motor.current, actuator.motor.torqueConst,
												  actuator.spindle.radius);
		ROS_INFO_THROTTLE(1,"electric current: %.5f, speed: %.5f, force %.5f", actuator.motor.current, actuator.spindle.angVel, actuatorForce);

		actuator.gear.position += actuator.spindle.angVel*period.nsec*1e-9;
        tendon.tendonLength = tendon.initialTendonLength - actuator.spindle.radius*actuator.gear.position;

        for(int i = 0; i < viaPoints.size(); i++)
        {
            if(viaPoints[i].prevPoint && viaPoints[i].nextPoint)
            {
                viaPoints[i].fa = viaPoints[i].prevPoint->fb;
                viaPoints[i].fb = viaPoints[i].prevPoint->fb;
            } else if (!viaPoints[i].prevPoint){
                viaPoints[i].fa = 0;
                //use this to compare with old functionality of plugin
                viaPoints[i].fb = actuator.elasticForce + actuatorForce;
                //viaPoint[i].fb = tendon.see.force;
            } else if (!viaPoints[i].nextPoint){
                viaPoints[i].fa = viaPoints[i].prevPoint->fb;
                viaPoints[i].fb = 0;
            }
            viaPoints[i].CalculateForce();
        };
	}
}
// make it a plugin loadable via pluginlib
PLUGINLIB_EXPORT_CLASS(roboy_simulation::MusclePlugin, roboy_simulation::MusclePlugin)
