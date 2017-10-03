#ifndef ROBOT_LOADER_IF_H
#define ROBOT_LOADER_IF_H

#include <QString>
#include <QVector>

#include <Box2D/Box2D.h>

#include "robot.h"

class RobotLoader_If
{
public:
    virtual QString loadRobotFile(QString file, Robot*& output/*, QMap<QString, pluginFactory>*/) = 0;
};

#endif // ROBOT_LOADER_IF_H
