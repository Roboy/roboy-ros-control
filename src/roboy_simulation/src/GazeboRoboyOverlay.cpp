#include "GazeboRoboyOverlay.hpp"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GazeboRoboyOverlay)

/////////////////////////////////////////////////
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

    // Create a push button, and connect it to the OnButton function
    QPushButton *button = new QPushButton(tr("Spawn Sphere"));
    connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));

    // Add the button to the frame's layout
    frameLayout->addWidget(button);

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
    this->resize(120, 30);

    // Create a node for transportation
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();
    this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
}

/////////////////////////////////////////////////
GazeboRoboyOverlay::~GazeboRoboyOverlay() {
}

/////////////////////////////////////////////////
void GazeboRoboyOverlay::OnButton() {
    msgs::Model model;
    model.set_name("plugin_unit_sphere_" + std::to_string(this->counter++));
    msgs::Set(model.mutable_pose(), ignition::math::Pose3d(0, 0, 1.5, 0, 0, 0));
    const double mass = 1.0;
    const double radius = 0.5;
    msgs::AddSphereLink(model, mass, radius);

    std::ostringstream newModelStr;
    newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << msgs::ModelToSDF(model)->ToString("")
    << "</sdf>";

    // Send the model to the gazebo server
    msgs::Factory msg;
    msg.set_sdf(newModelStr.str());
    this->factoryPub->Publish(msg);
}
