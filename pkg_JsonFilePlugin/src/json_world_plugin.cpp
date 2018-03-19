#include "json_world_plugin.h"
#include "json_world.h"

Json_World_Plugin::Json_World_Plugin()
{

}

QVector<WorldLoader_If*> Json_World_Plugin::getLoaders()
{
    return QVector<WorldLoader_If*>{ new JsonWorldLoader };
}

QVector<WorldSaver_If*> Json_World_Plugin::getSavers()
{
    return QVector<WorldSaver_If*>{ new JsonWorldSaver };
}
