#pragma once
// common definitions
#include "CommonDefinitions.h"
#include "CommunicationData.h"
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Pose.hh>
// ros
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <CommunicationData.h>
// messages
#include <std_msgs/Float32.h>
//std
#include <math.h>
#include <map>
#include <stdio.h>
#include <sstream>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>

enum MUSCLE_TYPE{
    EXTENSOR,
    FLEXOR,
    STABILIZER
};

namespace roboy_simulation {
    using namespace std;
    using namespace gazebo;

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

    struct SEE {
        double stiffness = 1.0;
        double length = 0.1;
        double lengthRest;
    };

    struct tendonType {
        vector<math::Vector3> MidPoint;
        vector<math::Vector3> Vector;
        //might need it to calculate length
        vector<math::Vector3> Orientation;
        vector<double> Pitch;
        vector<double> Roll;
    };

    class ITendon {
    public:
        void GetTendonInfo(vector<math::Vector3> &viaPointPos, tendonType *tendon_p);
        SEE see;
    private:
        ////////////////////////////////////////
        /// \brief Calculate the dot product between two vectors
        /// \param[in] _v1 vector 1 coordinates
        /// \param[in] _v2 vector 2 coordinates
        /// \return Dot product
        double DotProduct(const math::Vector3 &_v1, const math::Vector3 &_v2);

        ////////////////////////////////////////
        /// \brief Calculate the angle between two vectors
        /// \param[in] _v1 vector 1 coordinates
        /// \param[in] _v2 vector 2 coordinates
        /// \return Angle between two vectors in radians
        double Angle(const math::Vector3 &_v1, const math::Vector3 &_v2);
    };

    struct MyoMuscleInfo{
        string name;
        vector<gazebo::physics::LinkPtr> links;
        gazebo::physics::JointPtr spanning_joint;
        MUSCLE_TYPE muscle_type;
        uint link_index;
        vector<math::Vector3> viaPoints;
        Motor motor;
        Gear gear;
        Spindle spindle;
        SEE see;
    };

    class DummyMusclePlugin{

    public:
        DummyMusclePlugin();

        void Init(MyoMuscleInfo &myoMuscle, int roboyID);
        void Update(ros::Time &time, ros::Duration &period );
        string name;
        vector<gazebo::physics::LinkPtr> links;
        gazebo::physics::JointPtr joint;
        math::Vector3 momentArm;
        uint link_index;
        vector<math::Vector3> viaPoints;
        vector<math::Vector3> viaPointsInGlobalFrame;
        vector<math::Vector3> force;
        MUSCLE_TYPE muscle_type;
        double F_max = 100.0;
        double cmd = 0.0;
    private:
        ros::NodeHandlePtr nh;
        ros::Publisher actuatorForce_pub;
        int roboyID;
        double actuatorForce;
        ITendon tendon;
        transport::NodePtr node;
        transport::PublisherPtr visPub;
    };
}

