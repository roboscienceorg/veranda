#include "default_robot_loader_plugin.h"
#include "default_robot_loader.h"

Default_Robot_Loader_Plugin::Default_Robot_Loader_Plugin()
{

}

QVector<WorldObjectLoader_If*> Default_Robot_Loader_Plugin::getLoaders()
{
    return QVector<WorldObjectLoader_If*>{ new DefaultRobotLoader };
}
