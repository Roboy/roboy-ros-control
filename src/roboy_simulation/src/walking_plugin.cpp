#include <std_msgs/Bool.h>
#include <CommonDefinitions.h>
#include "walking_plugin.hpp"

WalkingPlugin::WalkingPlugin(QWidget *parent)
        : rviz::Panel(parent){
    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

    QHBoxLayout *roboyIdlayout = new QHBoxLayout();
    QLineEdit *roboyIDlabel= new QLineEdit(tr("roboy ID:"));
    roboyIDlabel->setReadOnly(true);
    roboyIdlayout->addWidget(roboyIDlabel);

    QComboBox *roboyID = new QComboBox();
    roboyID->setObjectName("roboyID");
    connect(roboyID, SIGNAL(currentIndexChanged(int)), this, SLOT(changeID(int)));
    roboyIdlayout->addWidget(roboyID);

    frameLayout->addLayout(roboyIdlayout);

    QHBoxLayout *stancelegstatelayout = new QHBoxLayout();
    QHBoxLayout *liftofflegstatelayout = new QHBoxLayout();
    QHBoxLayout *swinglegstatelayout = new QHBoxLayout();
    QHBoxLayout *stancepreplegstatelayout = new QHBoxLayout();

    QHBoxLayout *stancelayoutleft = new QHBoxLayout();
    LightWidget *leftlight0 = new LightWidget();
    leftlight0->setObjectName("StanceLeft");
    QLineEdit *stanceleftleg= new QLineEdit(tr("Stance"));
    stanceleftleg->setReadOnly(true);
    stancelayoutleft->addWidget(leftlight0);
    stancelayoutleft->addWidget(stanceleftleg);

    QHBoxLayout *liftofflayoutleft = new QHBoxLayout();
    LightWidget *leftlight1 = new LightWidget();
    leftlight1->setObjectName("LiftOffLeft");
    QLineEdit *liftoffleftleg= new QLineEdit(tr("Lift-Off"));
    liftoffleftleg->setReadOnly(true);
    liftofflayoutleft->addWidget(leftlight1);
    liftofflayoutleft->addWidget(liftoffleftleg);

    QHBoxLayout *swinglayoutleft = new QHBoxLayout();
    LightWidget *leftlight2 = new LightWidget();
    leftlight2->setObjectName("SwingLeft");
    QLineEdit *swingleftleg= new QLineEdit(tr("Swing"));
    swingleftleg->setReadOnly(true);
    swinglayoutleft->addWidget(leftlight2);
    swinglayoutleft->addWidget(swingleftleg);

    QHBoxLayout *stancepreplayoutleft = new QHBoxLayout();
    LightWidget *leftlight3 = new LightWidget();
    leftlight3->setObjectName("StancePrepLeft");
    QLineEdit *stanceprepleftleg= new QLineEdit(tr("Stance Preparation"));
    stanceprepleftleg->setReadOnly(true);
    stancepreplayoutleft->addWidget(leftlight3);
    stancepreplayoutleft->addWidget(stanceprepleftleg);

    stancelegstatelayout->addLayout(stancelayoutleft);
    liftofflegstatelayout->addLayout(liftofflayoutleft);
    swinglegstatelayout->addLayout(swinglayoutleft);
    stancepreplegstatelayout->addLayout(stancepreplayoutleft);

    QHBoxLayout *stancelayoutright = new QHBoxLayout();
    LightWidget *rightlight0 = new LightWidget();
    rightlight0->setObjectName("StanceRight");
    QLineEdit *stancerightleg= new QLineEdit(tr("Stance"));
    stancerightleg->setReadOnly(true);
    stancelayoutright->addWidget(rightlight0);
    stancelayoutright->addWidget(stancerightleg);

    QHBoxLayout *liftofflayoutright = new QHBoxLayout();
    LightWidget *rightlight1 = new LightWidget();
    rightlight1->setObjectName("LiftOffRight");
    QLineEdit *liftoffrightleg= new QLineEdit(tr("Lift-Off"));
    liftoffrightleg->setReadOnly(true);
    liftofflayoutright->addWidget(rightlight1);
    liftofflayoutright->addWidget(liftoffrightleg);

    QHBoxLayout *swinglayoutright = new QHBoxLayout();
    LightWidget *rightlight2 = new LightWidget();
    rightlight2->setObjectName("SwingRight");
    QLineEdit *swingrightleg= new QLineEdit(tr("Swing"));
    swingrightleg->setReadOnly(true);
    swinglayoutright->addWidget(rightlight2);
    swinglayoutright->addWidget(swingrightleg);

    QHBoxLayout *stancepreplayoutright = new QHBoxLayout();
    LightWidget *rightlight3 = new LightWidget();
    rightlight3->setObjectName("StancePrepRight");
    QLineEdit *stancepreprightleg= new QLineEdit(tr("Stance Preparation"));
    stancepreprightleg->setReadOnly(true);
    stancepreplayoutright->addWidget(rightlight3);
    stancepreplayoutright->addWidget(stancepreprightleg);

    stancelegstatelayout->addLayout(stancelayoutright);
    liftofflegstatelayout->addLayout(liftofflayoutright);
    swinglegstatelayout->addLayout(swinglayoutright);
    stancepreplegstatelayout->addLayout(stancepreplayoutright);

    frameLayout->addLayout(stancelegstatelayout);
    frameLayout->addLayout(liftofflegstatelayout);
    frameLayout->addLayout(swinglegstatelayout);
    frameLayout->addLayout(stancepreplegstatelayout);

    QPushButton *initwalkcontroller = new QPushButton(tr("initialize WalkController"));
    initwalkcontroller->setObjectName("initwalkcontroller");
    connect(initwalkcontroller, SIGNAL(clicked()), this, SLOT(initWalkController()));
    frameLayout->addWidget(initwalkcontroller);

    QPushButton *shutdowncontroller = new QPushButton(tr("shutdown WalkController"));
    connect(shutdowncontroller, SIGNAL(clicked()), this, SLOT(shutDownWalkController()));
    frameLayout->addWidget(shutdowncontroller);

    QCheckBox *visualizeMesh= new QCheckBox(tr("show mesh"));
    visualizeMesh->setObjectName("visualizeMesh");
    connect(visualizeMesh, SIGNAL(clicked()), this, SLOT(showMesh()));
    frameLayout->addWidget(visualizeMesh);

    QCheckBox *visualizeTendon= new QCheckBox(tr("show tendon"));
    visualizeTendon->setObjectName("visualizeTendon");
    connect(visualizeTendon, SIGNAL(clicked()), this, SLOT(showTendon()));
    frameLayout->addWidget(visualizeTendon);

    QCheckBox *visualizeCOM = new QCheckBox(tr("show COM"));
    visualizeCOM->setObjectName("visualizeCOM");
    connect(visualizeCOM, SIGNAL(clicked()), this, SLOT(showCOM()));
    frameLayout->addWidget(visualizeCOM);

    QCheckBox *visualizeForce = new QCheckBox(tr("show force"));
    visualizeForce->setObjectName("visualizeForce");
    connect(visualizeForce, SIGNAL(clicked()), this, SLOT(showForce()));
    frameLayout->addWidget(visualizeForce);

    QCheckBox *visualizeMomentArm = new QCheckBox(tr("show momentArms"));
    visualizeMomentArm->setObjectName("visualizeMomentArm");
    connect(visualizeMomentArm, SIGNAL(clicked()), this, SLOT(showMomentArm()));
    frameLayout->addWidget(visualizeMomentArm);

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
    init_walk_controller_pub = nh->advertise<std_msgs::Bool>("/roboy/init_walk_controller", 1);
    id_sub = nh->subscribe("/roboy/id", 1000, &WalkingPlugin::updateId, this);
}

WalkingPlugin::~WalkingPlugin(){
    delete nh;
    delete spinner;
}

void WalkingPlugin::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void WalkingPlugin::load(const rviz::Config &config) {
    rviz::Panel::load(config);
}

void WalkingPlugin::initWalkController(){
    std_msgs::Bool msg;
    msg.data = true;
    init_walk_controller_pub.publish(msg);
    QPushButton* w = this->findChild<QPushButton*>("initwalkcontroller");
    w->setText("fuck me");
}

void WalkingPlugin::shutDownWalkController(){
    std_msgs::Bool msg;
    msg.data = false;
    init_walk_controller_pub.publish(msg);
}

void WalkingPlugin::showCOM() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeCOM");
    roboy_simulation::VisualizationControl msg;
    msg.id = currentID.second;
    msg.control = COM;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showForce() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeForce");
    roboy_simulation::VisualizationControl msg;
    msg.id = currentID.second;
    msg.control = Force;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showTendon() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeTendon");
    roboy_simulation::VisualizationControl msg;
    msg.id = currentID.second;
    msg.control = Tendon;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showMesh() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeMesh");
    roboy_simulation::VisualizationControl msg;
    msg.id = currentID.second;
    msg.control = Mesh;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showMomentArm() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeMomentArm");
    roboy_simulation::VisualizationControl msg;
    msg.id = currentID.second;
    msg.control = MomentArm;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::changeID(int index){
    QComboBox* roboyID = this->findChild<QComboBox*>("roboyID");
    currentID = make_pair(index, roboyID->currentText().toInt());
}

void WalkingPlugin::updateLegStates(const roboy_simulation::LegState::ConstPtr &msg){
    if(msg->id == currentID.second) {
        LightWidget *stance;
        LightWidget *lift;
        LightWidget *swing;
        LightWidget *prep;
        switch (msg->leg) {
            case LEG::LEFT: {
                stance = this->findChild<LightWidget *>("StanceLeft");
                lift = this->findChild<LightWidget *>("LiftOffLeft");
                swing = this->findChild<LightWidget *>("SwingLeft");
                prep = this->findChild<LightWidget *>("StancePrepLeft");
                break;
            }
            case LEG::RIGHT: {
                stance = this->findChild<LightWidget *>("StanceRight");
                lift = this->findChild<LightWidget *>("LiftOffRight");
                swing = this->findChild<LightWidget *>("SwingRight");
                prep = this->findChild<LightWidget *>("StancePrepRight");
                break;
            }
        }
        switch (msg->state) {
            case LEG_STATE::Stance: {
                stance->turnOn();
                lift->turnOff();
                swing->turnOff();
                prep->turnOff();
                break;
            }
            case LEG_STATE::Lift_off: {
                stance->turnOff();
                lift->turnOn();
                swing->turnOff();
                prep->turnOff();
                break;
            }
            case LEG_STATE::Swing: {
                stance->turnOff();
                lift->turnOff();
                swing->turnOn();
                prep->turnOff();
                break;
            }
            case LEG_STATE::Stance_Preparation: {
                stance->turnOff();
                lift->turnOff();
                swing->turnOff();
                prep->turnOn();
                break;
            }
        }
        stance->repaint();
        lift->repaint();
        swing->repaint();
        prep->repaint();
    }
}

void WalkingPlugin::updateId(const std_msgs::Int32::ConstPtr &msg){
    QComboBox* roboyID = this->findChild<QComboBox*>("roboyID");
    roboyID->addItem(QString::number(msg->data));
    leg_state_sub[msg->data] = nh->subscribe("/roboy/leg_state", 1000, &WalkingPlugin::updateLegStates, this);
    roboyID->repaint();
    // republish visualization
    showMesh();
    showCOM();
    showMomentArm();
    showForce();
    showTendon();
}


PLUGINLIB_EXPORT_CLASS(WalkingPlugin, rviz::Panel)