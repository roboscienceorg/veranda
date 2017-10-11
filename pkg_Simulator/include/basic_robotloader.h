#ifndef BASIC_ROBOTLOADER_H
#define BASIC_ROBOTLOADER_H

#include "interfaces/robot_loader_if.h"

class BasicRobotLoader : public RobotLoader_If
{
    virtual QString loadRobotFile(QString file, Robot*& output/*, QMap<QString, pluginFactory>*/) override;
};
#endif // BASIC_ROBOTLOADER_H
