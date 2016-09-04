#include <thread>
#include "walkTrainer.hpp"

WalkTrainer::WalkTrainer(){
    // Create a new transport node
    node = transport::NodePtr(new transport::Node());

    // Initialize the node with the world name
    node->Init("walkTrainer");

    // Create a publisher on the ~/physics topic
    transport::PublisherPtr physicsPub = node->Advertise<msgs::Physics>("~/physics");

    // set the physic params
    msgs::Physics physicsMsg;
    physicsMsg.set_type(msgs::Physics::ODE);
    physicsMsg.set_max_step_size(0.0003);
    physicsPub->Publish(physicsMsg);

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "WalkTrainer", ros::init_options::NoSigintHandler);
    }

    nh = new ros::NodeHandle;
};

WalkTrainer::~WalkTrainer() {
    delete nh;
}

void WalkTrainer::initializeWorlds(uint numberOfWorlds){
    // load numberOfWorlds empty worlds
    for(uint i=0;i<numberOfWorlds;i++) {
        world.push_back(gazebo::loadWorld("worlds/empty.world"));
    }
    gazebo::sensors::run_once(true);
    gazebo::sensors::run_threads();

    // load the legs in each world
    for(uint i=0;i<numberOfWorlds;i++) {
        world[i]->InsertModelFile("model://legs_with_muscles_simplified");

        // wait until the model is loaded
        int modelCountBefore = world[i]->GetModelCount();
        int retry = 0;
        while (world[i]->GetModelCount() == modelCountBefore) {
            gazebo::runWorld(world[i], 100);
            gazebo::common::Time::MSleep(100);
            retry++;
            if (retry > 1000)
                break;
        }
        if (world[i]->GetModelCount() == modelCountBefore + 1) {
            std::cout << "Successfully inserted model.\n";
        } else {
            std::cout << "Failed inserting model\n";
        }
    }
}

void WalkTrainer::simulate(){
    physics::ModelPtr model0 = world[0]->GetModel("legs_with_muscles_simplified");
    physics::ModelPtr model1 = world[1]->GetModel("legs_with_muscles_simplified");

    for(uint i=0;i<2;i++) {
        gazebo::runWorld(world[i], (i+1)*100);
    }
    if (model0 == nullptr && model1 == nullptr)
        cout << "could not find legs" << endl;
    else {
        cout << "world 0: " << model0->GetLink("foot_left")->GetWorldPose() << endl;
        cout << "world 1: " << model1->GetLink("foot_left")->GetWorldPose() << endl;
    }
}

int main(int _argc, char **_argv) {
    // setup Gazebo server
    if (gazebo::setupServer()) {
        std::cout << "Gazebo server setup successful\n";
    } else {
        std::cout << "Gazebo server setup failed!\n";
    }

    WalkTrainer walkTrainer;

    walkTrainer.initializeWorlds(1);

    while(ros::ok()){
        for(uint i=0;i<walkTrainer.world.size();i++) {
            gazebo::sensors::run_once(true);
            gazebo::runWorld(walkTrainer.world[i], 100);
        }
    }

    gazebo::shutdown();
}