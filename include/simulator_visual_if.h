#ifndef SIMULATOR_VISUAL_H
#define SIMULATOR_VISUAL_H

#include <QOpenGLWidget>

#include "robot.h"

class Simulator_Visual_If : public QOpenGLWidget
{
public:
    Simulator_Visual_If(QObject* parent = nullptr) : QOpenGLWidget(parent){}

public slots:
    void robotAddedToSimulation(Robot_Visual* robot, robot_id rId);
    void robotRemovedFromSimulation(robot_id rId);
    void mapObjectsSetInSimulation(/*QVector<[world-space polygons]> polygons*/);
}
#endif // SIMULATOR_VISUAL_H
