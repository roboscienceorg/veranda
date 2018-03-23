#ifndef JSON_OBJECT
#define JSON_OBJECT

#include "json_common.h"

class JsonObjectLoader : public WorldObjectLoader_If
{
    virtual QVector<QString> fileExts() { return QVector<QString>{"Json file (*.json)"}; }
    //need to fill this out
    virtual bool canLoadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If*> plugins){ return true; }
    virtual void getUserOptions(QString /*filePath*/, QMap<QString, WorldObjectComponent_Plugin_If*> /*plugins*/){}
    virtual WorldObject* loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);
};

class JsonObjectSaver : public WorldObjectSaver_If
{
    virtual QVector<QString> fileExts() { return QVector<QString>{"Json file (*.json)"}; }
    virtual void saveFile(QString filePath, WorldObject* objects);
};

#endif
