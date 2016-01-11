#include "ros/ros.h"
#include "common_utilities/Trajectory.h"
#include "common_utilities/InitializeRequest.h"
#include "common_utilities/InitializeResponse.h"
#include "common_utilities/Status.h"
#include "common_utilities/Steer.h"
#include <vector>

using namespace std;

class GUI{
public:
    GUI();
    
    ~GUI();
    
    // publish functions
    void initRequest(vector<signed char> motor);
    bool sendTrajectory(uint motor, uint32_t sampleRate, uint8_t controlMode, vector<float> setpoints);
    
    // callback functions
    void initResponseCallback(common_utilities::InitializeResponse msg);
    void statusCallback(common_utilities::Status msg);
    
private:
    ros::Publisher initPublisher;
    vector<ros::Publisher> trajectoryPublisher;
    ros::Subscriber initResponse, statusResponse;
    ros::NodeHandle nh;
};
