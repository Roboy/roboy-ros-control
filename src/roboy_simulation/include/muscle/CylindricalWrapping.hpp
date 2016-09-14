#ifndef _GAZEBO_CYLINDRICALWRAPPING_HPP_
#define _GAZEBO_CYLINDRICALWRAPPING_HPP_

#include "StateMachine.hpp"
#include "IViaPoints.hpp"

#include <boost/numeric/odeint.hpp>

namespace roboy_simulation
{
	using namespace gazebo;

	class ITendon;

	class CylindricalWrapping : public IViaPoints
	{

    public:
        CylindricalWrapping();
        CylindricalWrapping(math::Vector3 point, physics::LinkPtr link);
        CylindricalWrapping(math::Vector3 point, double radius, int state, int counter, physics::LinkPtr link);

        ////////////////////
        /// \brief This function updates the position of the attachment point.
        ///
        /// Retrives the links position from Gazebo.
        virtual void UpdateForcePoints();

        ////////////////////
        /// \brief This function applies the muscle force to the attachment point
        virtual void CalculateForce();

    public:
        StateMachine stateMachine;
        double radius;
        math::Vector3 prevCoord;
        math::Vector3 nextCoord;
        math::Vector3 prevCoordPlane;
        math::Vector3 nextCoordPlane;
        math::Vector3 prevForcePointPlane;
        math::Vector3 nextForcePointPlane;
        math::Vector3 normal;
        double arcAngle;

	};
}

#endif
