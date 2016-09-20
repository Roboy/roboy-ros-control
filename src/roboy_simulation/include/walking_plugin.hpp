#pragma once

#ifndef Q_MOC_RUN
// qt
#include <QPainter>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTableWidget>
#include <QComboBox>
#include <QTimer>
// std
#include <map>
// ros
#include <ros/ros.h>
#include <rviz/panel.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
//messages
#include "roboy_simulation/VisualizationControl.h"
#include "roboy_simulation/LegState.h"
#include "roboy_simulation/ControllerParameters.h"
#include <std_srvs/Trigger.h>
#include <std_msgs/Int32.h>
#include "roboy_simulation/Abortion.h"
#include "roboy_simulation/MotorControl.h"
// common definitions
#include "CommonDefinitions.h"
#endif

using namespace std;

class LightWidget : public QWidget {
Q_OBJECT
public:
    LightWidget(QWidget *parent = 0)
            : QWidget(parent), m_on(false) {
        this->setFixedWidth(20);
        this->setFixedHeight(20);
    }

    bool isOn() const { return m_on; }

    void setOn(bool on) {
        if(on)
            m_color = Qt::green;
        else
            m_color = Qt::gray;
        if (on == m_on)
            return;
        m_on = on;
    }

    void turnOff() { setOn(false); }

    void turnOn() { setOn(true); }

    void setColor(Qt::GlobalColor color){m_color = color;}

protected:
    virtual void paintEvent(QPaintEvent *)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setBrush(m_color);
        painter.drawEllipse(0, 0, width(), height());
    }

private:
    QColor m_color;
    bool m_on;
};

class WalkingPlugin : public rviz::Panel{
Q_OBJECT

public:
    WalkingPlugin(QWidget *parent = 0);

    ~WalkingPlugin();

    /**
     * Load all configuration data for this panel from the given Config object.
     * @param config rviz config file
     */
    virtual void load(const rviz::Config &config);

    /**
     * Save all configuration data from this panel to the given
     * Config object.  It is important here that you call save()
     * on the parent class so the class id and panel name get saved.
     * @param config rviz config file
     */
    virtual void save(rviz::Config config) const;

public Q_SLOTS:

    void toggleWalkController();

    void shutDownWalkController();

    void showCOM();

    void showForce();

    void showTendon();

    void showMesh();

    void showMomentArm();

    void showStateMachineParameters();

    void showCoordinateSystems();

    void showForceTorqueSensors();

    void changeID(int index);

    void updateSimulationState(const roboy_simulation::ControllerParameters::ConstPtr &msg);

    void resetWorld();

    void play();

    void pause();

    void slowMotion();

    void updateInteractiveMarker();

    void sendMotorControl();

    void refresh();

private:
    void updateLegStates(const roboy_simulation::LegState::ConstPtr &msg);

    void updateId(const std_msgs::Int32::ConstPtr &msg);

    void abortion(const roboy_simulation::Abortion::ConstPtr &msg);



    ros::NodeHandle *nh;
    pair<uint, uint> currentID;
    map<uint, ros::Subscriber> leg_state_sub;
    ros::AsyncSpinner *spinner;
    ros::Publisher roboy_visualization_control_pub, toggle_walk_controller_pub, sim_control_pub, motor_control_pub;
    ros::Subscriber id_sub, simulation_state_sub, abort_sub;
    ros::ServiceClient reset_world_srv;

    ros::Timer frame_timer;
};