#ifndef _GAZEBO_IACTUATOR_HPP_
#define _GAZEBO_IACTUATOR_HPP_

#include <boost/numeric/odeint.hpp>

namespace gazebo{

    struct Motor {
		double current = 0.0;
		double torqueConst = 1.0;
		double resistance = 100.0;
		double inductance = 100.0;
		double voltage = 0.0;
		double BEMFConst = 0.001488; // back electromagnetic force constant
		double inertiaMoment = 4.09;
	};

	struct Gear {
		double inertiaMoment = 0.4;
		double ratio = 53;
		double efficiency = 0.59; // gear efficciency
		double appEfficiency; // approximated efficiency
		double position = 0.0;
	};

	struct Spindle {
		double angVel = 0.0; // angular velocity of the spindle
		double radius = 0.005;
	};

    class IActuator {
		// state vector for differential model
	public:
		typedef std::vector<double> state_type;
		// private: std::vector< double > x(2);

		// stepper for integration
		boost::numeric::odeint::runge_kutta4<state_type> stepper;

		////////////////////////////////////////
		/// \brief Approximates gear's velocity
		/// according to the direction of the rotation of the gear, i.e.
		/// eta or 1/eta
		/// \return Approximated value for gear efficiency
		double EfficiencyApproximation();

		void DiffModel(const state_type &x, state_type &dxdt, const double /* t */);

		Motor motor;
		Gear gear;
		Spindle spindle;
		double elasticForce;
	};
}

#endif
