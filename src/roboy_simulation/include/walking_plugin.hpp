#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rviz/panel.h>
#include <stdio.h>
#include <QPainter>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.h>
#include "roboy_simulation/VisualizationControl.h"

#endif

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

    void showCOM();
    void showForce();
    void showTendon();

private:
    ros::NodeHandle *nh;
    ros::AsyncSpinner *spinner;
    ros::Publisher roboy_visualization_control_pub;

    enum{
        Tendon,
        COM,
        Force
    }visualization;

    bool visualizeTendon = false, visualizeCOM = false, visualizeForce = false;
};