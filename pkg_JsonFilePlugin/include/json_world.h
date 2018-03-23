#ifndef JSON_WORLD
#define JSON_WORLD

#include "json_common.h"

class JsonWorldLoader : public WorldLoader_If
{
    virtual QVector<QString> fileExts() { return QVector<QString>{"Json file (*.json)"}; }
    //need to fill this out
    virtual bool canLoadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If*> plugins){ return true; }
    virtual void getUserOptions(QString /*filePath*/, QMap<QString, WorldObjectComponent_Plugin_If*> /*plugins*/){}
    virtual QVector<WorldObject*> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);
};

class JsonWorldSaver : public WorldSaver_If
{
    virtual QVector<QString> fileExts() { return QVector<QString>{"Json file (*.json)"}; }
    virtual void saveFile(QString filePath, QVector<WorldObject*> objects);
};

#endif
