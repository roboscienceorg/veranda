#ifndef WORLD_OBJECT_SAVER_IF_H
#define WORLD_OBJECT_SAVER_IF_H

#include <QString>
#include <QVector>
#include "world_object.h"
#include "dllapi.h"

class WorldObjectSaver_If
{
public:
    ~WorldObjectSaver_If(){}

    virtual QVector<QString> fileExts() = 0;
    virtual void saveFile(QString filePath, WorldObject* objects) = 0;
};

class WorldSaver_If
{
public:
    ~WorldSaver_If(){}

    virtual QVector<QString> fileExts() = 0;
    virtual void saveFile(QString filePath, QVector<WorldObject*> objects) = 0;
};

#endif
