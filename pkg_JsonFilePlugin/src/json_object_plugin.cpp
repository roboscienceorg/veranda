#include "json_object_plugin.h"
#include "json_object.h"

Json_Object_Plugin::Json_Object_Plugin()
{

}

QVector<WorldLoader_If*> Json_Object_Plugin::getLoaders()
{
    return QVector<WorldLoader_If*>{ new JsonObjectLoader };
}

QVector<WorldLoader_If*> Json_Object_Plugin::getSavers()
{
    return QVector<WorldLoader_If*>{ new JsonObjectSaver };
}
