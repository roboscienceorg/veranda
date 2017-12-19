#ifndef WORLD_OBJECT_LOADER_IF_H
#define WORLD_OBJECT_LOADER_IF_H

#include <QString>
#include <QVector>
#include "world_object.h"

class WorldObjectLoader_If
{
public:
    ~WorldObjectLoader_If(){}

    virtual QVector<QString> fileExts() = 0;
    virtual QVector<WorldObject> loadFile(QString filePath, QMap<QString, WorldObjectComponent_If*> plugins) = 0;
};

#endif
