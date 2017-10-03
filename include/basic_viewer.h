#ifndef BASIC_VIEWER_H
#define BASIC_VIEWER_H

#include "interfaces/simulator_visual_if.h"

class BasicViewer : public Simulator_Visual_If
{
    Q_OBJECT

public:
    BasicViewer(QWidget* parent = nullptr);

public slots:

    //A robot was selected as the 'current' robot
    void robotSelected(robot_id rId) override;

    //Robot added to simulation
    //Do not delete the robot interface when the robot is removed; it will be handled elsewhere
    void robotAddedToSimulation(Robot_Visual* robot, robot_id rId) override;

    //Robot removed from simulation
    void robotRemovedFromSimulation(robot_id rId) override;

    //Map objects were set into the simulation
    void mapObjectsSetInSimulation(/*QVector<[world-space polygons]> polygons*/) override;
};

#endif // BASIC_VIEWER_H
