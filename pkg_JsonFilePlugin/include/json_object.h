#ifndef JSON_OBJECT_LOADER
#define JSON_OBJECT_LOADER

#include <QVector>
#include <QString>

#include <sdsmt_simulator/object_loader_if.h>
#include <sdsmt_simulator/object_saver_if.h>

class JsonObjectLoader : public WorldObjectLoader_If
{
    virtual QVector<QString> fileExts() { return QVector<QString>{""};}
    virtual bool canLoadFile(QString /*filePath*/){ return true; }
    virtual void getUserOptions(QString /*filePath*/){}
    virtual WorldObject* loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);
};

#endif
