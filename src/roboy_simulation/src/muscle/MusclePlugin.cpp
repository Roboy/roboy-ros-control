#include "MusclePlugin.hpp"

namespace roboy_simulation {

	MusclePlugin::MusclePlugin() {
		x.resize(2);
	}

	void MusclePlugin::Init(MyoMuscleInfo &myoMuscle) {
		//state initialization
		x[0] = 0.0;
		x[1] = 0.0;
		actuator.motor.voltage = 0.0;
		actuator.spindle.angVel = 0;
		viaPoints = myoMuscle.viaPoints;
		//linked list
        for(int i = 0; i < viaPoints.size(); i++){
            if(i>0)
            {
                viaPoints[i].prevPoint = &(viaPoints[i-1]);
                viaPoints[i-1].nextPoint = &(viaPoints[i]);
            }
        }
		actuator.motor = myoMuscle.motor;
		actuator.gear = myoMuscle.gear;
		actuator.spindle = myoMuscle.spindle;
		tendon.see = myoMuscle.see;
        name = myoMuscle.name;
		tendon.see.expansion = 0.0;
		tendon.see.force = 0.0;
	}

	void MusclePlugin::Update( ros::Time &time, ros::Duration &period ) {

		// TODO: calculate PID result
//		pid.calc_output(cmd,pos,period);
		actuator.motor.voltage = cmd;

        for(int i = 0; i < viaPoints.size(); i++){
            // absolute position + relative position=actual position of each via point
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
		}, x, time.toSec(), period.toSec());

		actuator.motor.current = x[0];
		actuator.spindle.angVel = x[1];

		//calculate motor force
		actuatorForce = actuator.ElectricMotorModel(actuator.motor.current, actuator.motor.torqueConst,
												  actuator.spindle.radius);
		ROS_INFO_THROTTLE(1,"electric current: %.5f, speed: %.5f, force %.5f", actuator.motor.current, actuator.spindle.angVel, actuatorForce);

		actuator.gear.position += actuator.spindle.angVel*period.toSec();
        tendon.tendonLength = tendon.initialTendonLength - actuator.spindle.radius*actuator.gear.position;
	}
}
// make it a plugin loadable via pluginlib
PLUGINLIB_EXPORT_CLASS(roboy_simulation::MusclePlugin, roboy_simulation::MusclePlugin)
