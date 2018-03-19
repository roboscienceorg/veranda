#ifndef JSON_WORLD
#define JSON_WORLD

#include <QVector>
#include <QString>

#include <QJsonObject>
#include <QJsonArray>

#include <sdsmt_simulator/object_loader_if.h>
#include <sdsmt_simulator/object_saver_if.h>

class JsonWorldLoader : public WorldLoader_If
{
    virtual QVector<QString> fileExts() { return QVector<QString>{"Json file (*.json)"}; }
    //need to fill this out
    virtual bool canLoadFile(QString /*filePath*/){ return true; }
    virtual void getUserOptions(QString /*filePath*/){}
    virtual QVector<WorldObject*> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);
};

class JsonWorldSaver : public WorldSaver_If
{
    virtual QVector<QString> fileExts() { return QVector<QString>{"Json file (*.json)"}; }
    virtual void saveFile(QString filePath, QVector<WorldObject*> objects);
};

#endif
