#ifndef DEFAULT_ROBOT_LOADER
#define DEFAULT_ROBOT_LOADER

#include <QVector>
#include <QString>

#include <sdsmt_simulator/world_object_loader_if.h>

class DefaultRobotLoader : public WorldObjectLoader_If
{
    virtual QVector<QString> fileExts() { return QVector<QString>{""};}
    virtual QVector<WorldObject *> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);

    WorldObject* makeDiffDriveBot(QMap<QString, WorldObjectComponent_Plugin_If*> plugins);
    WorldObject* makeAckermannBot(QMap<QString, WorldObjectComponent_Plugin_If*> plugins);
};

#endif
