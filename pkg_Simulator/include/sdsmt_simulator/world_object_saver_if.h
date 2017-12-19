#ifndef WORLD_OBJECT_SAVER_IF_H
#define WORLD_OBJECT_SAVER_IF_H

#include <QString>
#include <QVector>
#include "world_object.h"

class WorldObjectSaver_If
{
public:
    ~WorldObjectSaver_If(){}

    virtual QVector<QString> fileExts() = 0;
    virtual void saveFile(QString filePath, QVector<WorldObject*> objects) = 0;
};

#endif
