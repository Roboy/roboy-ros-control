#include "IViaPoints.hpp"

using namespace roboy_simulation;

IViaPoints::IViaPoints(): linkPosition(math::Vector3(0,0,0)), linkRotation(math::Quaternion(0,0,0,0)), localCoordinates(math::Vector3(0,0,0)),
                            globalCoordinates(math::Vector3(0,0,0)), type(Type::FIXPOINT), prevPoint(nullptr), nextPoint(nullptr),
                            prevForcePoint(math::Vector3(0,0,0)), nextForcePoint(math::Vector3(0,0,0)), fa(0), fb(0), prevForce(math::Vector3(0,0,0)),
                            nextForce(math::Vector3(0,0,0)), previousSegmentLength(0)
{

};

IViaPoints::IViaPoints(math::Vector3 point): IViaPoints()
{
    localCoordinates = point;
};

IViaPoints::IViaPoints(math::Vector3 point, Type t): IViaPoints(point)
{
    type = t;
};

void IViaPoints::UpdateForcePoints()
{
    prevForcePoint = globalCoordinates;
    nextForcePoint = globalCoordinates;
    previousSegmentLength = (prevPoint)?((prevPoint->nextForcePoint - this->prevForcePoint).GetLength()):(0);
};

void IViaPoints::CalculateForce()
{

    if(prevPoint && nextPoint)
    {
        //TODO: change fa, fb with respect to friction
        // need to know if before or behind see
        //previousSegmentKiteLineVelocity needed
    }

    if (prevPoint)
    {
        math::Vector3 A = prevPoint->nextForcePoint - this->prevForcePoint;
        prevForce = A/A.GetLength() * fa;
        //link->AddForceAtRelativePosition(Fa, this->prevForcePoint);
    }
    else if (nextPoint)
    {
        math::Vector3 B = nextPoint->prevForcePoint - this->nextForcePoint;
        nextForce = B/B.GetLength() * fb;
        //link->AddForceAtRelativePosition(Fb, this->nextForcePoint);
    }
};
