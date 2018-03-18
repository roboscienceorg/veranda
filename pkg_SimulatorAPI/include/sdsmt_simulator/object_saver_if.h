#ifndef WORLD_OBJECT_SAVER_IF_H
#define WORLD_OBJECT_SAVER_IF_H

#include <QString>
#include <QVector>
#include <QFile>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
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

class JsonWorldSaver : public WorldSaver_If
{
public:
    QVector<QString> fileExts()
    {
        QVector<QString> exts;
        exts.append("json");
        return exts;
    }

    void saveFile(QString filePath, QVector<WorldObject*> objects)
    {
        QFile saveFile(filePath);
        QJsonObject everything;
        QJsonArray worldObjectArray;
        foreach(WorldObject* obj, objects)
        {
            QJsonObject worldObject;
            obj->writeJson(worldObject);
            worldObjectArray.append(worldObject);
        }
        everything["worldObjects"] = worldObjectArray;
        QJsonDocument saveDoc(everything);
        saveFile.write(saveDoc.toJson());
    }
};

#endif
