#include "IActuator.hpp"

using namespace gazebo;

double IActuator::EfficiencyApproximation() {
    double param1 = 0.1; // defines steepness of the approximation
	double param2 = 0; // defines zero crossing of the approximation
	return gear.efficiency + (1 / gear.efficiency - gear.efficiency) *
								 (0.5 * (tanh(-param1 * spindle.angVel * motor.current - param2) + 1));
}

	double IActuator::ElectricMotorModel(const double _current, const double _torqueConstant,
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
