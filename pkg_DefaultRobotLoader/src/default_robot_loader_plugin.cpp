#include "default_robot_loader_plugin.h"
#include "default_robot_loader.h"

Default_Robot_Loader_Plugin::Default_Robot_Loader_Plugin()
{

}

QVector<WorldLoader_If*> Default_Robot_Loader_Plugin::getLoaders()
{
    return QVector<WorldLoader_If*>{ new DefaultRobotLoader };
}
