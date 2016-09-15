#ifndef _GAZEBO_STATE_MACHINE_HPP_
#define _GAZEBO_STATE_MACHINE_HPP_

//#include "Definitions.hh"

#include <gazebo/physics/physics.hh>
#include <gazebo/math/Vector3.hh>
#include <math.h>

namespace roboy_simulation
{
    using namespace gazebo;

    class StateMachine{

    public:
        ////////////////////
        /// \brief The state enum for different states of the state machine
        ///
        /// Differentiates between three possible states of the state machine:
        /// 0: Not Wrapping
        /// 1: Positive, for wrapping over angles smaller than 180°
        /// 2: Negative, for wrapping over angles larger than 180°
        enum State
        {
            NOTWRAPPING = 0,
            POSITIVE = 1,
            NEGATIVE = 2
        };

    public:
        StateMachine();

        ////////////////////////////////////////
        /// \brief Decides if the muscle is positive, negative or not wrapping
        /// \param[in] prevPoint the previous attachment point
        /// \param[in] nextPoint the nextattachment point
        /// \param[in] center the center of the wrapping surface
        /// \param[in] radius the radius of the wrapping surface
        void UpdateState(math::Vector3& prevPoint, math::Vector3& nextPoint, math::Vector3& center, double radius);

        ////////////////////////////////////////
        /// \brief updates the RevCounter
        /// \param[in] projection the current projection from vector prevForcePoint->nextForcePoint onto the vector prevForcePoint->prevPoint
        void UpdateRevCounter(double projection);

    public:
        State state;
        int revCounter;
    private:
        bool firstUpdate;
        math::Vector3 normal;
        double projection;
    };
}

#endif
