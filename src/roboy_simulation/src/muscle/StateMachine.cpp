#include "StateMachine.hpp"

using namespace roboy_simulation;

StateMachine::StateMachine(): firstUpdate(true), state(NOTWRAPPING), normal(math::Vector3(0,0,0)), revCounter(0), projection(0)
{

};

void StateMachine::UpdateState(math::Vector3& prevPoint, math::Vector3& nextPoint, math::Vector3& center,double radius)
{
    //compute unit vectors and according length
    double l_j1 = (prevPoint-center).GetLength();
    math::Vector3 j1 = (prevPoint-center)/l_j1;
    double l_j2 = (nextPoint-center).GetLength();
    math::Vector3 j2 = (nextPoint-center)/l_j2;

    //compute normal
    math::Vector3 normal = j1.Cross(j2);

    //calculate height = distance between straight line from previous point to next point and sphere center
    math::Vector3 diff = prevPoint-nextPoint;
    double height = l_j1 * sin(acos((j1).Dot(diff/diff.GetLength())));

    //if(counter%update == 0)
    //{
    //    gzdbg << "height: " << height << "\n";
    //}

    if(firstUpdate)
    {
        this->normal = normal;
        switch (state)
        {
            case NOTWRAPPING:
                projection = 0.0;
                break;
            default:
                if (revCounter % 2 == 0)
                {
                    projection = -1.0;
                }
                else
                {
                    projection = 1.0;
                }
                break;
        }
    }

    //state machine decides how muscle length is calculated
    switch(state)
    {
        case NOTWRAPPING:
            if((height < radius) && (j1.Dot(j2) < 0))
            {
                state = POSITIVE;
            }
            break;
        case POSITIVE:
            if ((height >= radius) && (revCounter == 0))
            {
                state = NOTWRAPPING;
            }
            else if (normal.Dot(this->normal) < 0)
            {
                state = NEGATIVE;
            }
            break;
        case NEGATIVE:
            if (normal.Dot(this->normal) < 0)
            {
                state = POSITIVE;
            }
            break;
    }

    this->normal = normal;
    firstUpdate = false;
};

void StateMachine::UpdateRevCounter(double proj)
{
    if(state == POSITIVE)
    {
        if(projection<0 && proj>0)
        {
            revCounter--;
        }
        else if (projection>0 && proj<0)
        {
            revCounter++;
        }
    }
    else if (state == NEGATIVE)
    {
        if(projection<0 && proj>0)
        {
            revCounter++;
        }
        else if (projection>0 && proj<0)
        {
            revCounter--;
        }
    }

    projection = proj;
};
