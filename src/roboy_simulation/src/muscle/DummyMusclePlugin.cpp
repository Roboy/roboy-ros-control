#include "DummyMusclePlugin.hpp"

namespace roboy_simulation {

    DummyMusclePlugin::DummyMusclePlugin() {
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "DummyMusclePlugin",
                      ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
    }

    void ITendon::GetTendonInfo(vector<math::Vector3> &viaPointPos, tendonType *tendon_p)//try later with pointer
    {
        see.length = 0;
        for (int i = 0; i < viaPointPos.size() - 1; i++) {
            tendon_p->MidPoint.push_back((viaPointPos[i] + viaPointPos[i + 1]) / 2);
            tendon_p->Vector.push_back(viaPointPos[i] - viaPointPos[i + 1]);
            double length = tendon_p->Vector[i].GetLength();
            tendon_p->Orientation.push_back(tendon_p->Vector[i].Normalize());
            tendon_p->Pitch.push_back(atan(tendon_p->Orientation[i][0] / tendon_p->Orientation[i][2]));
            tendon_p->Roll.push_back(
                    -acos(sqrt((pow(tendon_p->Orientation[i][0], 2) + pow(tendon_p->Orientation[i][2], 2)))));
            see.length += length;
        }
    }

    double ITendon::DotProduct(const math::Vector3 &_v1, const math::Vector3 &_v2) {
        return _v1.x * _v2.x + _v1.y * _v2.y + _v1.z * _v2.z;
    }


    double ITendon::Angle(const math::Vector3 &_v1, const math::Vector3 &_v2) {
        return acos(_v1.Dot(_v2) / _v1.GetLength() * _v2.GetLength());
    }

    void DummyMusclePlugin::Init(MyoMuscleInfo &myoMuscle, int id) {
        roboyID = id;
        link_index = myoMuscle.link_index;
        links = myoMuscle.links;
        joint = myoMuscle.spanning_joint;
        muscle_type = myoMuscle.muscle_type;
        viaPoints = myoMuscle.viaPoints;
        viaPointsInGlobalFrame = myoMuscle.viaPoints;
        force = myoMuscle.viaPoints;
        name = myoMuscle.name;
        tendon.see = myoMuscle.see;

        char topic[100];
        snprintf(topic, 100, "/roboy%d/%s/actuatorForce", roboyID, name.c_str());
        actuatorForce_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
    }

    void DummyMusclePlugin::Update( ros::Time &time, ros::Duration &period ) {
        tendonType newTendon;

        uint j = 0;
        for (uint i = 0; i < viaPointsInGlobalFrame.size(); i++) {
            // absolute position + relative position=actual position of each via point
            gazebo::math::Pose linkPose = links[j]->GetWorldPose();
            viaPointsInGlobalFrame[i] = linkPose.pos + linkPose.rot.RotateVector(viaPoints[i]);
            if(i==link_index-1){
                momentArm = (viaPointsInGlobalFrame[i]-joint->GetWorldPose().pos).Cross(
                        (viaPointsInGlobalFrame[i+1]-viaPointsInGlobalFrame[i]).Normalize());
                j++;
            }
        }

        tendon.GetTendonInfo(viaPointsInGlobalFrame, &newTendon);

        actuatorForce = cmd;

        std_msgs::Float32 msg;
        msg.data = actuatorForce;
        actuatorForce_pub.publish(msg);
        ros::spinOnce();

        for (uint i = 0; i < viaPointsInGlobalFrame.size(); i++) {
            force[i] = newTendon.Orientation[i];
            force[i] *= actuatorForce;
            ROS_INFO_THROTTLE(1.0, "force: %f orientation: %f %f %f", actuatorForce, newTendon.Orientation[i].x,
                              newTendon.Orientation[i].y, newTendon.Orientation[i].z);
        }
    }
}
// make it a plugin loadable via pluginlib
PLUGINLIB_EXPORT_CLASS(roboy_simulation::DummyMusclePlugin, roboy_simulation::DummyMusclePlugin)