#ifndef ROBOT_LOADER_IF_H
#define ROBOT_LOADER_IF_H

#include <QString>

#include "robot.h"

class RobotLoader_If
{
public:
    virtual ~RobotLoader_If(){}

    virtual Robot* loadRobotFile(QString file) = 0;
};

#endif // ROBOT_LOADER_IF_H
