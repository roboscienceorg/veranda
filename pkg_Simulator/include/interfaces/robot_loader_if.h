#ifndef ROBOT_LOADER_IF_H
#define ROBOT_LOADER_IF_H

#include <QString>

#include "robot.h"

class RobotLoader_If
{
public:
    virtual ~RobotLoader_If(){}
    virtual QString loadRobotFile(QString file, Robot*& output) = 0;
};

#endif // ROBOT_LOADER_IF_H
