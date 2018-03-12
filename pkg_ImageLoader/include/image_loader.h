#ifndef DEFAULT_ROBOT_LOADER
#define DEFAULT_ROBOT_LOADER

#include <QVector>
#include <QString>
#include <QSharedPointer>

#include <sdsmt_simulator/object_loader_if.h>
#include <sdsmt_simulator/world_object.h>
#include <Box2D/Box2D.h>

#include "optiondialog.h"

class ImageLoader : public WorldLoader_If
{
private:
    QSharedPointer<ImageOptions> lastOptions;
    QVector<QVector<b2PolygonShape *> > getShapesFromFile(QString filePath);

public:
    virtual bool canLoadFile(QString filePath);
    virtual void getUserOptions(QString filePath);
    virtual QVector<QString> fileExts() { return QVector<QString>{"Black and White Image (*.png *.jpg *.jpeg *.bmp)"};}
    virtual QVector<WorldObject*> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);
};

#endif
