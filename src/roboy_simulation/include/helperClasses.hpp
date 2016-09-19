#pragma once

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

class CoordinateSystem{
public:
    CoordinateSystem(gazebo::physics::LinkPtr link):m_link(link){};
    void Update(){
        gazebo::math::Pose pose = m_link->GetWorldPose();
        rot = pose.rot;
        origin = pose.pos;
        X = pose.rot.RotateVector(gazebo::math::Vector3::UnitX);
        Y = pose.rot.RotateVector(gazebo::math::Vector3::UnitY);
        Z = pose.rot.RotateVector(gazebo::math::Vector3::UnitZ);
    }
    void UpdateHeading(){
        gazebo::math::Pose pose = m_link->GetWorldPose();
        gazebo::math::Quaternion q(0,0,pose.rot.GetAsEuler().z);
        origin = pose.pos;
        X = q.RotateVector(gazebo::math::Vector3::UnitX);
        Y = q.RotateVector(gazebo::math::Vector3::UnitY);
        Z = q.RotateVector(gazebo::math::Vector3::UnitZ);
    }
    gazebo::math::Vector3 origin, X, Y, Z;
    gazebo::math::Quaternion rot;
private:
    gazebo::physics::LinkPtr m_link;
};

typedef boost::shared_ptr<CoordinateSystem> CoordSys;