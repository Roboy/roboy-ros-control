#include "walking_plugin.hpp"

WalkingPlugin::WalkingPlugin(QWidget *parent)
        : rviz::Panel(parent){
    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

    QCheckBox *visualizeTendon= new QCheckBox(tr("show tendon"));
    connect(visualizeTendon, SIGNAL(clicked()), this, SLOT(showTendon()));
    frameLayout->addWidget(visualizeTendon);

    QCheckBox *visualizeCOM = new QCheckBox(tr("show COM"));
    connect(visualizeCOM, SIGNAL(clicked()), this, SLOT(showCOM()));
    frameLayout->addWidget(visualizeCOM);

    QCheckBox *visualizeForce = new QCheckBox(tr("show force"));
    connect(visualizeForce, SIGNAL(clicked()), this, SLOT(showForce()));
    frameLayout->addWidget(visualizeForce);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "GazeboRoboyOverlay",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = new ros::NodeHandle;

    spinner = new ros::AsyncSpinner(1);

    roboy_visualization_control_pub = nh->advertise<roboy_simulation::VisualizationControl>("/roboy/visualization_control", 1);
}

WalkingPlugin::~WalkingPlugin(){
    delete nh;
    delete spinner;
}

void WalkingPlugin::showTendon() {
    visualizeTendon = !visualizeTendon;
    roboy_simulation::VisualizationControl msg;
    msg.control = Tendon;
    msg.value = visualizeTendon;
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showCOM() {
    visualizeCOM = !visualizeCOM;
    roboy_simulation::VisualizationControl msg;
    msg.control = COM;
    msg.value = visualizeCOM;
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showForce() {
    visualizeForce = !visualizeForce;
    roboy_simulation::VisualizationControl msg;
    msg.control = Force;
    msg.value = visualizeForce;
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void WalkingPlugin::load(const rviz::Config &config) {
    rviz::Panel::load(config);
}

PLUGINLIB_EXPORT_CLASS(WalkingPlugin, rviz::Panel)