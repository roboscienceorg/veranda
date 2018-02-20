#ifndef DEFAULT_ROBOT_LOADER
#define DEFAULT_ROBOT_LOADER

#include <QVector>
#include <QString>

#include <sdsmt_simulator/object_loader_if.h>
#include <sdsmt_simulator/world_object.h>
#include <Box2D/Box2D.h>

class ImageLoader : public WorldLoader_If
{
private:
    QVector<QVector<b2PolygonShape *> > getShapesFromFile(QString filePath);

public:
    virtual QVector<QString> fileExts() { return QVector<QString>{"png", "jpg", "jpeg", "bmp"};}
    virtual QVector<WorldObject *> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);
};

#endif
