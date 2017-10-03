#ifndef SIMULATOR_VISUAL_H
#define SIMULATOR_VISUAL_H

#include <QWidget>

#include "robot_interfaces.h"

class Simulator_Visual_If : public QWidget
{
    Q_OBJECT

public:
    Simulator_Visual_If(QWidget* parent = nullptr) : QWidget(parent){}

signals:
    //Signals that the user clicked on a robot
    void userSelectedRobot(robot_id rId);

public slots:
    //A robot was selected as the 'current' robot
    virtual void robotSelected(robot_id rId) = 0;

    //Robot added to simulation
    //Do not delete the robot interface when the robot is removed; it will be handled elsewhere
    virtual void robotAddedToSimulation(Robot_Visual* robot, robot_id rId) = 0;

    //Robot removed from simulation
    virtual void robotRemovedFromSimulation(robot_id rId) = 0;

    //Map objects were set into the simulation
    virtual void mapObjectsSetInSimulation(/*QVector<[world-space polygons]> polygons*/) = 0;
};
#endif // SIMULATOR_VISUAL_H
