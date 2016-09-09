#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rviz/panel.h>
#include <stdio.h>
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
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.h>
#include "roboy_simulation/VisualizationControl.h"
#include "roboy_simulation/LegState.h"
#include "roboy_simulation/SimulationState.h"
#include "CommonDefinitions.h"
#include <std_msgs/Int32.h>
#include <map>
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
        if (on == m_on)
            return;
        m_on = on;
    }

    void turnOff() { setOn(false); }

    void turnOn() { setOn(true); }

protected:
    virtual void paintEvent(QPaintEvent *)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        if (m_on) {
            painter.setBrush(Qt::green);
        }else{
            painter.setBrush(Qt::gray);
        }
        painter.drawEllipse(0, 0, width(), height());
    }

private:
    QColor m_color;
    bool m_on;
};

class WalkingPlugin : public rviz::Panel {
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

    void initWalkController();

    void shutDownWalkController();

    void showCOM();

    void showForce();

    void showTendon();

    void showMesh();

    void showMomentArm();

    void changeID(int index);

    void updateSimulationState(const roboy_simulation::SimulationState::ConstPtr &msg);

private:
    void updateLegStates(const roboy_simulation::LegState::ConstPtr &msg);

    void updateId(const std_msgs::Int32::ConstPtr &msg);

    ros::NodeHandle *nh;
    pair<uint, uint> currentID;
    map<uint, ros::Subscriber> leg_state_sub;
//    map<uint, ros::Publisher> leg_state_sub;
    ros::AsyncSpinner *spinner;
    ros::Publisher roboy_visualization_control_pub, init_walk_controller_pub;
    ros::Subscriber id_sub, simulation_state_sub;
    enum {
        Tendon,
        COM,
        Force,
        MomentArm,
        Mesh
    } visualization;
};