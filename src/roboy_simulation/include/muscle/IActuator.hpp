#ifndef _GAZEBO_IACTUATOR_HPP_
#define _GAZEBO_IACTUATOR_HPP_

#include <boost/numeric/odeint.hpp>

namespace gazebo{

    struct Motor {
        double current = 0.0; // [A]
        double torqueConst = 11.2; // [V/s]
        double resistance = 0.797; // [Ohm]
        double inductance = 0.118e-03; // [H]
        double voltage = 0.0; // [V]
        double BEMFConst = 14.2e-03; // [Nm/A]
        double inertiaMoment = 4.09e-07; // [kgm^2]
	};

	struct Gear {
        double inertiaMoment = 0.4e-07; // [kgm^2]
        double ratio = 53; // [1]
        double efficiency = 0.59; // [1]
        double appEfficiency; // approximated efficiency
	};

	struct Spindle {
        double angVel = 0.0; // [1/s]
        double radius = 4.5e-03; // [m]
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
