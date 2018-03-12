#ifndef WORLD_OBJECT_LOADER_IF_H
#define WORLD_OBJECT_LOADER_IF_H

#include <QString>
#include <QVector>
#include "world_object_component_plugin.h"
#include "world_object.h"
#include "dllapi.h"

class WorldObjectLoader_If
{
public:
    ~WorldObjectLoader_If(){}

    virtual QVector<QString> fileExts() = 0;
    virtual bool canLoadFile(QString filePath) = 0;
    virtual void getUserOptions(QString /*filePath*/){}
    virtual WorldObject* loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If*> plugins) = 0;
};

class WorldLoader_If
{
public:
    ~WorldLoader_If(){}

    virtual QVector<QString> fileExts() = 0;
    virtual bool canLoadFile(QString filePath) = 0;
    virtual void getUserOptions(QString /*filePath*/){}
    virtual QVector<WorldObject*> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If*> plugins) = 0;
};

#endif
