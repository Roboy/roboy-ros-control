#include "ros/ros.h"
#include "gui_dummy/Trajectory.h"
#include "gui_dummy/InitializeRequest.h"
#include "gui_dummy/InitializeResponse.h"
#include "gui_dummy/Status.h"
#include "gui_dummy/Steer.h"
#include <vector>

using namespace std;

class GUI{
public:
    GUI();
    
    ~GUI();
    
    // publish functions
    void initRequest(vector<unsigned char> motor);
    bool sendTrajectory(uint motor, uint32_t sampleRate, uint8_t controlMode, vector<float> setpoints);
    
    // callback functions
    void initResponseCB(gui_dummy::InitializeResponse msg);
    void statusCB(gui_dummy::Status msg);
    
private:
    ros::Publisher initPublisher;
    vector<ros::Publisher> trajectoryPublisher;
    ros::Subscriber initResponse, statusResponse;
    ros::NodeHandle nh;
};
