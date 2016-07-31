#include "GazeboRoboyOverlay.hpp"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GazeboRoboyOverlay)

GazeboRoboyOverlay::GazeboRoboyOverlay()
        : GUIPlugin() {
    this->counter = 0;

    // Set the frame background and foreground colors
    this->setStyleSheet(
            "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    // Create the layout that sits inside the frame
    QVBoxLayout *frameLayout = new QVBoxLayout();

    QCheckBox *visualizeTendon = new QCheckBox(tr("show tendon"));
    connect(visualizeTendon, SIGNAL(clicked()), this, SLOT(showTendon()));
    frameLayout->addWidget(visualizeTendon);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);

    // Position and resize this widget
    this->move(10, 10);
    this->resize(120, 90);

    // Create a node for transportation
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();
    this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "GazeboRoboyOverlay",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = new ros::NodeHandle;

    visualizeTendon_pub = nh->advertise<std_msgs::Bool>("/visual/visualizeTendon", 1);
    visualizeForce_pub = nh->advertise<std_msgs::Bool>("/visual/visualizeForce", 1);
    visualizeCOM_pub = nh->advertise<std_msgs::Bool>("/visual/visualizeCOM", 1);
}

GazeboRoboyOverlay::~GazeboRoboyOverlay() {
    delete nh;
}

void GazeboRoboyOverlay::showTendon(){
    static bool showTendonFlag = false;
    showTendonFlag = !showTendonFlag;
    std_msgs::Bool msg;
    msg.data = showTendonFlag;
    visualizeTendon_pub.publish(msg);
    ros::spinOnce();
}