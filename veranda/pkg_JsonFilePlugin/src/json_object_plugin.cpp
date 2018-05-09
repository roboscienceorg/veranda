#include "json_object_plugin.h"
#include "json_object.h"
#include "json_world.h"

QVector<WorldObjectLoader_If*> Json_Object_Plugin::getLoaders()
{
    return QVector<WorldObjectLoader_If*>{ new JsonObjectLoader };
}

QVector<WorldObjectSaver_If*> Json_Object_Plugin::getSavers()
{
    return QVector<WorldObjectSaver_If*>{ new JsonObjectSaver };
}
