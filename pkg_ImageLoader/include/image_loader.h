#ifndef DEFAULT_ROBOT_LOADER
#define DEFAULT_ROBOT_LOADER

#include <QVector>
#include <QString>

#include <sdsmt_simulator/world_object_loader_if.h>
#include <sdsmt_simulator/world_object.h>
#include <Box2D/Box2D.h>

class ImageLoader : public WorldObjectLoader_If
{
private:
    QVector<QVector<b2PolygonShape *> > getShapesFromFile(QString filePath);

public:
    virtual QVector<QString> fileExts() { return QVector<QString>{""};}
    virtual QVector<WorldObject *> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);
};

#endif
