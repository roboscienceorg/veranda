#ifndef DEFAULT_ROBOT_LOADER
#define DEFAULT_ROBOT_LOADER

#include <QVector>
#include <QString>

#include <sdsmt_simulator/object_loader_if.h>
#include <sdsmt_simulator/object_saver_if.h>

class DefaultRobotLoader : public WorldLoader_If
{
    virtual QVector<QString> fileExts() { return QVector<QString>{""};}
    virtual bool canLoadFile(QString /*filePath*/){ return true; }
    virtual void getUserOptions(QString /*filePath*/){}
    virtual QVector<WorldObject*> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);

    WorldObject* makeDiffDriveBot(QMap<QString, WorldObjectComponent_Plugin_If*> plugins);
    WorldObject* makeAckermannBot(QMap<QString, WorldObjectComponent_Plugin_If*> plugins);
};

#endif
