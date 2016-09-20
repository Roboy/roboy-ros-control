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
    QLabel *roboyIDlabel= new QLabel(tr("roboy ID:"));
    roboyIdlayout->addWidget(roboyIDlabel);

    QComboBox *roboyID = new QComboBox();
    roboyID->setObjectName("roboyID");
    connect(roboyID, SIGNAL(currentIndexChanged(int)), this, SLOT(changeID(int)));
    roboyIdlayout->addWidget(roboyID);

    QPushButton *refreshbutton = new QPushButton(tr("refresh"));
    connect(refreshbutton, SIGNAL(clicked()), this, SLOT(refresh()));
    roboyIdlayout->addWidget(refreshbutton);

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

    QHBoxLayout *simcontrol = new QHBoxLayout();
    QHBoxLayout *options = new QHBoxLayout();
    QVBoxLayout *options0 = new QVBoxLayout();
    QVBoxLayout *options1 = new QVBoxLayout();

    QHBoxLayout *controllerOptions = new QHBoxLayout();

    QCheckBox *togglewalkcontroller = new QCheckBox(tr("toggle WalkController"));
    togglewalkcontroller->setObjectName("togglewalkcontroller");
    connect(togglewalkcontroller, SIGNAL(clicked()), this, SLOT(toggleWalkController()));
    controllerOptions->addWidget(togglewalkcontroller);

    LightWidget *status = new LightWidget();
    status->setObjectName("status");
    QLineEdit *statusString= new QLineEdit(tr("Controller OFF"));
    statusString->setReadOnly(true);
    statusString->setObjectName("statusString");
    controllerOptions->addWidget(status);
    controllerOptions->addWidget(statusString);
    frameLayout->addLayout(controllerOptions);

    QPushButton *button = new QPushButton(tr("reset"));
    connect(button, SIGNAL(clicked()), this, SLOT(resetWorld()));
    simcontrol->addWidget(button);

    button= new QPushButton(tr("play"));
    connect(button, SIGNAL(clicked()), this, SLOT(play()));
    simcontrol->addWidget(button);

    button = new QPushButton(tr("pause"));
    connect(button, SIGNAL(clicked()), this, SLOT(pause()));
    simcontrol->addWidget(button);

    button = new QPushButton(tr("slow motion"));
    connect(button, SIGNAL(clicked()), this, SLOT(slowMotion()));
    simcontrol->addWidget(button);

    button = new QPushButton(tr("update interactive marker"));
    connect(button, SIGNAL(clicked()), this, SLOT(updateInteractiveMarker()));
    simcontrol->addWidget(button);

    frameLayout->addLayout(simcontrol);

    QCheckBox *visualizeMesh= new QCheckBox(tr("show mesh"));
    visualizeMesh->setObjectName("visualizeMesh");
    connect(visualizeMesh, SIGNAL(clicked()), this, SLOT(showMesh()));
    options0->addWidget(visualizeMesh);

    QCheckBox *visualizeTendon= new QCheckBox(tr("show tendon"));
    visualizeTendon->setObjectName("visualizeTendon");
    connect(visualizeTendon, SIGNAL(clicked()), this, SLOT(showTendon()));
    options0->addWidget(visualizeTendon);

    QCheckBox *visualizeCOM = new QCheckBox(tr("show COM"));
    visualizeCOM->setObjectName("visualizeCOM");
    connect(visualizeCOM, SIGNAL(clicked()), this, SLOT(showCOM()));
    options0->addWidget(visualizeCOM);

    QCheckBox *visualizeForce = new QCheckBox(tr("show force"));
    visualizeForce->setObjectName("visualizeForce");
    connect(visualizeForce, SIGNAL(clicked()), this, SLOT(showForce()));
    options0->addWidget(visualizeForce);

    QCheckBox *visualizeMomentArm = new QCheckBox(tr("show momentArms"));
    visualizeMomentArm->setObjectName("visualizeMomentArm");
    connect(visualizeMomentArm, SIGNAL(clicked()), this, SLOT(showMomentArm()));
    options0->addWidget(visualizeMomentArm);

    QCheckBox *visualizeStateMachineParameters = new QCheckBox(tr("show state machine parameters"));
    visualizeStateMachineParameters->setObjectName("visualizeStateMachineParameters");
    connect(visualizeStateMachineParameters, SIGNAL(clicked()), this, SLOT(showStateMachineParameters()));
    options0->addWidget(visualizeStateMachineParameters);

    QCheckBox *visualizeCoordinateSystems = new QCheckBox(tr("show coordinate systems"));
    visualizeCoordinateSystems->setObjectName("visualizeCoordinateSystems");
    connect(visualizeCoordinateSystems, SIGNAL(clicked()), this, SLOT(showCoordinateSystems()));
    options1->addWidget(visualizeCoordinateSystems);

    QCheckBox *visualizeForceTorqueSensors = new QCheckBox(tr("show force torque sensors"));
    visualizeForceTorqueSensors->setObjectName("visualizeForceTorqueSensors");
    connect(visualizeForceTorqueSensors, SIGNAL(clicked()), this, SLOT(showForceTorqueSensors()));
    options1->addWidget(visualizeForceTorqueSensors);

    options->addLayout(options0);
    options->addLayout(options1);
    frameLayout->addLayout(options);

    QHBoxLayout *parameters = new QHBoxLayout();

    QTableWidget* table = new QTableWidget(36,2);
    table->setObjectName("table");

    QTableWidgetItem *item0 = new QTableWidgetItem("F_contact");
    QTableWidgetItem *item1 = new QTableWidgetItem("d_lift");
    QTableWidgetItem *item2 = new QTableWidgetItem("d_prep");
    QTableWidgetItem *item3 = new QTableWidgetItem("F_max");
    QTableWidgetItem *item4 = new QTableWidgetItem("psi_heading");
    QTableWidgetItem *item5 = new QTableWidgetItem("psi_heading");
    QTableWidgetItem *item6 = new QTableWidgetItem("omega_heading");
    QTableWidgetItem *item7 = new QTableWidgetItem("v_COM");
    QTableWidgetItem *item8 = new QTableWidgetItem("k_v");
    QTableWidgetItem *item9 = new QTableWidgetItem("k_h");
    QTableWidgetItem *item10 = new QTableWidgetItem("k_p_theta_left");
    QTableWidgetItem *item11 = new QTableWidgetItem("k_p_theta_right");
    QTableWidgetItem *item12 = new QTableWidgetItem("k_d_theta_left");
    QTableWidgetItem *item13 = new QTableWidgetItem("k_d_theta_right");
    QTableWidgetItem *item14 = new QTableWidgetItem("k_p_phi");
    QTableWidgetItem *item15 = new QTableWidgetItem("k_d_phi");
    QTableWidgetItem *item16 = new QTableWidgetItem("k_V");
    QTableWidgetItem *item17 = new QTableWidgetItem("k_P");
    QTableWidgetItem *item18 = new QTableWidgetItem("k_Q");
    QTableWidgetItem *item19 = new QTableWidgetItem("k_omega");
    QTableWidgetItem *item20 = new QTableWidgetItem("k_M_Fplus");
    QTableWidgetItem *item21 = new QTableWidgetItem("c_hip_lift");
    QTableWidgetItem *item22 = new QTableWidgetItem("c_knee_lift");
    QTableWidgetItem *item23 = new QTableWidgetItem("c_stance_lift");
    QTableWidgetItem *item24 = new QTableWidgetItem("c_swing_prep");
    QTableWidgetItem *item25 = new QTableWidgetItem("theta_groin_0");
    QTableWidgetItem *item26 = new QTableWidgetItem("phi_groin_0");
    QTableWidgetItem *item27 = new QTableWidgetItem("theta_trunk_0");
    QTableWidgetItem *item28 = new QTableWidgetItem("phi_trunk_0");
    QTableWidgetItem *item29 = new QTableWidgetItem("theta_knee");
    QTableWidgetItem *item30 = new QTableWidgetItem("theta_ankle");
    QTableWidgetItem *item31 = new QTableWidgetItem("d_s");
    QTableWidgetItem *item32 = new QTableWidgetItem("d_c");
    QTableWidgetItem *item33 = new QTableWidgetItem("v_s");
    QTableWidgetItem *item34 = new QTableWidgetItem("v_c");
    QTableWidgetItem *item35 = new QTableWidgetItem("sim_time[s]");
    table->setItem(0,0,item0);
    table->setItem(1,0,item1);
    table->setItem(2,0,item2);
    table->setItem(3,0,item3);
    table->setItem(4,0,item4);
    table->setItem(5,0,item5);
    table->setItem(6,0,item6);
    table->setItem(7,0,item7);
    table->setItem(8,0,item8);
    table->setItem(9,0,item9);
    table->setItem(10,0,item10);
    table->setItem(11,0,item11);
    table->setItem(12,0,item12);
    table->setItem(13,0,item13);
    table->setItem(14,0,item14);
    table->setItem(15,0,item15);
    table->setItem(16,0,item16);
    table->setItem(17,0,item17);
    table->setItem(18,0,item18);
    table->setItem(19,0,item19);
    table->setItem(20,0,item20);
    table->setItem(21,0,item21);
    table->setItem(22,0,item22);
    table->setItem(23,0,item23);
    table->setItem(24,0,item24);
    table->setItem(25,0,item25);
    table->setItem(26,0,item26);
    table->setItem(27,0,item27);
    table->setItem(28,0,item28);
    table->setItem(29,0,item29);
    table->setItem(30,0,item30);
    table->setItem(31,0,item31);
    table->setItem(32,0,item32);
    table->setItem(33,0,item33);
    table->setItem(34,0,item34);
    table->setItem(35,0,item35);

    parameters->addWidget(table);

    QVBoxLayout *motorcontrol0 = new QVBoxLayout();
    QVBoxLayout *motorcontrol1 = new QVBoxLayout();

    for(uint i=0; i<16; i++){
        QString m("motor");
        QLabel *label = new QLabel(m+QString::number(i));
        motorcontrol0->addWidget(label);
        QLineEdit* line = new QLineEdit("0");
        line->setObjectName(m+QString::number(i));
        connect(line, SIGNAL(editingFinished()), this, SLOT(sendMotorControl()));
        motorcontrol1->addWidget(line);
    }

    parameters->addLayout(motorcontrol0);
    parameters->addLayout(motorcontrol1);

    frameLayout->addLayout(parameters);

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
        ros::init(argc, argv, "RvizWalkingPlugin",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = new ros::NodeHandle;

    spinner = new ros::AsyncSpinner(1);

    roboy_visualization_control_pub = nh->advertise<roboy_simulation::VisualizationControl>("/roboy/visualization_control", 1);
    toggle_walk_controller_pub = nh->advertise<std_msgs::Bool>("/roboy/toggle_walk_controller", 1);
    id_sub = nh->subscribe("/roboy/id", 1, &WalkingPlugin::updateId, this);
    simulation_state_sub = nh->subscribe("/roboy/simulationState", 1, &WalkingPlugin::updateSimulationState, this);
    reset_world_srv = nh->serviceClient<std_srvs::Trigger>("/roboy/reset_world");
    abort_sub = nh->subscribe("/roboy/abort", 1, &WalkingPlugin::abortion, this);
    sim_control_pub = nh->advertise<std_msgs::Int32>("/roboy/sim_control", 1);
    motor_control_pub = nh->advertise<roboy_simulation::MotorControl>("/roboy/motor_control", 100);

    // create a timer to update the published transforms
//    frame_timer = nh->createTimer(ros::Duration(0.01), &WalkingPlugin::frameCallback, this);
}

WalkingPlugin::~WalkingPlugin(){
    delete nh;
    delete spinner;
}

void WalkingPlugin::save(rviz::Config config) const {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeMesh");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeTendon");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeCOM");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeForce");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeMomentArm");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeStateMachineParameters");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeCoordinateSystems");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeForceTorqueSensors");
    config.mapSetValue(w->objectName(), w->isChecked());
    rviz::Panel::save(config);
}

void WalkingPlugin::load(const rviz::Config &config) {
    rviz::Panel::load(config);
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeMesh");
    bool checked = false;
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeTendon");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeCOM");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeForce");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeMomentArm");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeStateMachineParameters");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeCoordinateSystems");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeForceTorqueSensors");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
}

void WalkingPlugin::toggleWalkController(){
    QCheckBox* w = this->findChild<QCheckBox*>("togglewalkcontroller");
    std_msgs::Bool msg;
    msg.data = w->isChecked();
    toggle_walk_controller_pub.publish(msg);
    LightWidget *status = this->findChild<LightWidget *>("status");
    QLineEdit* statusString = this->findChild<QLineEdit*>("statusString");
    if(w->isChecked()){
        status->setColor(Qt::green);
        statusString->setText("Controller ON");
    }else{
        status->setColor(Qt::gray);
        statusString->setText("Controller OFF");
    }

}

void WalkingPlugin::shutDownWalkController(){
    std_msgs::Bool msg;
    msg.data = false;
//    init_walk_controller_pub.publish(msg);
}

void WalkingPlugin::showCOM() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeCOM");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = COM;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showForce() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeForce");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = Forces;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showTendon() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeTendon");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = Tendon;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showMesh() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeMesh");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = Mesh;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showMomentArm() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeMomentArm");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = MomentArm;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showStateMachineParameters(){
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeStateMachineParameters");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = StateMachineParameters;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showCoordinateSystems(){
//    QCheckBox* w = this->findChild<QCheckBox*>("visualizeCoordinateSystems");
//    roboy_simulation::VisualizationControl msg;
//    msg.roboyID = currentID.second;
//    msg.control = CoordinateSystems;
//    msg.value = w->isChecked();
//    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::showForceTorqueSensors(){
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeForceTorqueSensors");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = ForceTorqueSensors;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void WalkingPlugin::changeID(int index){
    QComboBox* roboyID = this->findChild<QComboBox*>("roboyID");
    currentID = make_pair(index, roboyID->currentText().toInt());
    // republish visualization
    showMesh();
    showCOM();
    showMomentArm();
    showForce();
    showTendon();
    toggleWalkController();
    showStateMachineParameters();
}

void WalkingPlugin::updateSimulationState(const roboy_simulation::ControllerParameters::ConstPtr &msg){
    if(msg->roboyID == currentID.second) {
        QTableWidgetItem *item0 = new QTableWidgetItem(QString::number(msg->F_contact));
        QTableWidgetItem *item1 = new QTableWidgetItem(QString::number(msg->d_lift));
        QTableWidgetItem *item2 = new QTableWidgetItem(QString::number(msg->d_prep));
        QTableWidgetItem *item3 = new QTableWidgetItem(QString::number(msg->F_max));
        QTableWidgetItem *item4 = new QTableWidgetItem(QString::number(msg->psi_heading));
        QTableWidgetItem *item5 = new QTableWidgetItem(QString::number(msg->psi_heading));
        QTableWidgetItem *item6 = new QTableWidgetItem(QString::number(msg->omega_heading));
        QTableWidgetItem *item7 = new QTableWidgetItem(QString::number(msg->v_COM));
        QTableWidgetItem *item8 = new QTableWidgetItem(QString::number(msg->k_v));
        QTableWidgetItem *item9 = new QTableWidgetItem(QString::number(msg->k_h));
        char str[200];
        sprintf(str, "{%lf, %lf, %lf, %lf}", msg->k_p_theta_left[0], msg->k_p_theta_left[1],
                msg->k_p_theta_left[2], msg->k_p_theta_left[3]);
        QTableWidgetItem *item10 = new QTableWidgetItem(str);
        sprintf(str, "{%lf, %lf, %lf, %lf}", msg->k_p_theta_right[0], msg->k_p_theta_right[1],
                msg->k_p_theta_right[2], msg->k_p_theta_right[3]);
        QTableWidgetItem *item11 = new QTableWidgetItem(str);
        sprintf(str, "{%lf, %lf, %lf, %lf}", msg->k_d_theta_left[0], msg->k_d_theta_left[1],
                msg->k_d_theta_left[2], msg->k_d_theta_left[3]);
        QTableWidgetItem *item12 = new QTableWidgetItem(str);
        sprintf(str, "{%lf, %lf, %lf, %lf}", msg->k_d_theta_right[0], msg->k_d_theta_right[1],
                msg->k_d_theta_right[2], msg->k_d_theta_right[3]);
        QTableWidgetItem *item13 = new QTableWidgetItem(str);
        sprintf(str, "{%lf, %lf}", msg->k_p_phi[0], msg->k_p_phi[1]);
        QTableWidgetItem *item14 = new QTableWidgetItem(str);
        sprintf(str, "{%lf, %lf}", msg->k_d_phi[0], msg->k_d_phi[1]);
        QTableWidgetItem *item15 = new QTableWidgetItem(str);
        QTableWidgetItem *item16 = new QTableWidgetItem(QString::number(msg->k_V));
        QTableWidgetItem *item17 = new QTableWidgetItem(QString::number(msg->k_P));
        QTableWidgetItem *item18 = new QTableWidgetItem(QString::number(msg->k_Q));
        QTableWidgetItem *item19 = new QTableWidgetItem(QString::number(msg->k_omega));
        QTableWidgetItem *item20 = new QTableWidgetItem(QString::number(msg->k_M_Fplus));
        QTableWidgetItem *item21 = new QTableWidgetItem(QString::number(msg->c_hip_lift));
        QTableWidgetItem *item22 = new QTableWidgetItem(QString::number(msg->c_knee_lift));
        QTableWidgetItem *item23 = new QTableWidgetItem(QString::number(msg->c_stance_lift));
        QTableWidgetItem *item24 = new QTableWidgetItem(QString::number(msg->c_swing_prep));
        sprintf(str, "{%lf, %lf}", msg->theta_groin_0[0], msg->theta_groin_0[1]);
        QTableWidgetItem *item25 = new QTableWidgetItem(str);
        sprintf(str, "{%lf, %lf}", msg->phi_groin_0[0], msg->phi_groin_0[1]);
        QTableWidgetItem *item26 = new QTableWidgetItem(str);
        QTableWidgetItem *item27 = new QTableWidgetItem(QString::number(msg->theta_trunk_0));
        QTableWidgetItem *item28 = new QTableWidgetItem(QString::number(msg->phi_trunk_0));
        sprintf(str, "{%lf, %lf}", msg->theta_knee[0], msg->theta_knee[1]);
        QTableWidgetItem *item29 = new QTableWidgetItem(str);
        sprintf(str, "{%lf, %lf}", msg->theta_ankle[0], msg->theta_ankle[1]);
        QTableWidgetItem *item30 = new QTableWidgetItem(str);
        sprintf(str, "{%lf, %lf}", msg->d_s[0], msg->d_s[1]);
        QTableWidgetItem *item31 = new QTableWidgetItem(str);
        sprintf(str, "{%lf, %lf}", msg->d_c[0], msg->d_c[1]);
        QTableWidgetItem *item32 = new QTableWidgetItem(str);
        sprintf(str, "{%lf, %lf}", msg->v_s[0], msg->v_s[1]);
        QTableWidgetItem *item33 = new QTableWidgetItem(str);
        sprintf(str, "{%lf, %lf}", msg->v_c[0], msg->v_c[1]);
        QTableWidgetItem *item34 = new QTableWidgetItem(str);
        QTableWidgetItem *item35 = new QTableWidgetItem(QString::number(msg->sim_time));
        QTableWidget* table = this->findChild<QTableWidget*>("table");
        table->setItem(0,1,item0);
        table->setItem(1,1,item1);
        table->setItem(2,1,item2);
        table->setItem(3,1,item3);
        table->setItem(4,1,item4);
        table->setItem(5,1,item5);
        table->setItem(6,1,item6);
        table->setItem(7,1,item7);
        table->setItem(8,1,item8);
        table->setItem(9,1,item9);
        table->setItem(10,1,item10);
        table->setItem(11,1,item11);
        table->setItem(12,1,item12);
        table->setItem(13,1,item13);
        table->setItem(14,1,item14);
        table->setItem(15,1,item15);
        table->setItem(16,1,item16);
        table->setItem(17,1,item17);
        table->setItem(18,1,item18);
        table->setItem(19,1,item19);
        table->setItem(20,1,item20);
        table->setItem(21,1,item21);
        table->setItem(22,1,item22);
        table->setItem(23,1,item23);
        table->setItem(24,1,item24);
        table->setItem(25,1,item25);
        table->setItem(26,1,item26);
        table->setItem(27,1,item27);
        table->setItem(28,1,item28);
        table->setItem(29,1,item29);
        table->setItem(30,1,item30);
        table->setItem(31,1,item31);
        table->setItem(32,1,item32);
        table->setItem(33,1,item33);
        table->setItem(34,1,item34);
        table->setItem(35,1,item35);
    }
}

void WalkingPlugin::resetWorld(){
    LightWidget *status = this->findChild<LightWidget *>("status");
    QLineEdit* statusString = this->findChild<QLineEdit*>("statusString");
    status->setColor(Qt::gray);
    statusString->setText("Controller OFF");
    status->repaint();
    std_srvs::Trigger srv;
    reset_world_srv.call(srv);
}

void WalkingPlugin::play(){
    std_msgs::Int32 msg;
    msg.data = Play;
    sim_control_pub.publish(msg);
}

void WalkingPlugin::pause(){
    std_msgs::Int32 msg;
    msg.data = Pause;
    sim_control_pub.publish(msg);
}

void WalkingPlugin::slowMotion(){
    std_msgs::Int32 msg;
    msg.data = Slow_Motion;
    sim_control_pub.publish(msg);
}

void WalkingPlugin::updateInteractiveMarker(){
    std_msgs::Int32 msg;
    msg.data = UpdateInteractiveMarker;
    sim_control_pub.publish(msg);
}

void WalkingPlugin::sendMotorControl(){
    roboy_simulation::MotorControl msg;
    msg.roboyID = currentID.second;
    QString m("motor");
    for(uint i=0; i<16; i++){
        QLineEdit* line = this->findChild<QLineEdit *>(m+QString::number(i));
        bool ok = false;
        msg.voltage.push_back(line->text().toFloat(&ok));
        if(!ok)
            line->setText("invalid value");
    }
    motor_control_pub.publish(msg);
}

void WalkingPlugin::refresh(){
    showCOM();
    showCoordinateSystems();
    showForce();
    showForceTorqueSensors();
    showMesh();
    showMomentArm();
    showStateMachineParameters();
    showTendon();
}

void WalkingPlugin::updateLegStates(const roboy_simulation::LegState::ConstPtr &msg){
    if(msg->roboyID == currentID.second) {
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
    int index = roboyID->findText(QString::number(msg->data));
    if(index==-1) {
        roboyID->addItem(QString::number(msg->data));
        leg_state_sub[msg->data] = nh->subscribe("/roboy/leg_state", 2, &WalkingPlugin::updateLegStates, this);
        roboyID->repaint();
        // republish visualization
        showMesh();
        showCOM();
        showMomentArm();
        showForce();
        showTendon();
        toggleWalkController();
        showStateMachineParameters();
    }
}

void WalkingPlugin::abortion(const roboy_simulation::Abortion::ConstPtr &msg){
    if(msg->roboyID==currentID.second) {
        LightWidget *status;
        QLineEdit *statusString;
        switch (msg->reason) {
            case COMheight:
                status = this->findChild<LightWidget *>("status");
                status->setColor(Qt::red);
                statusString = this->findChild<QLineEdit *>("statusString");
                statusString->setText("com height below threshold");
                break;
            case headingDeviation:
                status = this->findChild<LightWidget *>("status");
                status->setColor(Qt::red);
                statusString = this->findChild<QLineEdit *>("statusString");
                statusString->setText("heading deviation above threshold");
                break;
            case selfCollision:
                status = this->findChild<LightWidget *>("status");
                status->setColor(Qt::red);
                statusString = this->findChild<QLineEdit *>("statusString");
                statusString->setText("self collision detected");
                break;
        }
        status->repaint();
    }
}

PLUGINLIB_EXPORT_CLASS(WalkingPlugin, rviz::Panel)