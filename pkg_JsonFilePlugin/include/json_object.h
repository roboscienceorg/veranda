#ifndef JSON_OBJECT
#define JSON_OBJECT

#include <QVector>
#include <QString>

#include "json_common.h"

#include <sdsmt_simulator/object_loader_if.h>
#include <sdsmt_simulator/object_saver_if.h>

class JsonObjectLoader : public WorldObjectLoader_If
{
    virtual QVector<QString> fileExts() { return QVector<QString>{"Json file (*.json)"}; }
    //need to fill this out
    virtual bool canLoadFile(QString /*filePath*/){ return true; }
    virtual void getUserOptions(QString /*filePath*/){}
    virtual WorldObject* loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);
};

class JsonObjectSaver : public WorldObjectSaver_If
{
    virtual QVector<QString> fileExts() { return QVector<QString>{"Json file (*.json)"}; }
    virtual void saveFile(QString filePath, WorldObject* objects);
};

#endif
