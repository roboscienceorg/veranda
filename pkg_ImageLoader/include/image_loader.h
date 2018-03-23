#ifndef DEFAULT_ROBOT_LOADER
#define DEFAULT_ROBOT_LOADER

#include <QVector>
#include <QString>
#include <QSharedPointer>

#include <sdsmt_simulator/object_loader_if.h>
#include <sdsmt_simulator/world_object.h>
#include <Box2D/Box2D.h>

#include "optiondialog.h"
#include "imageparser.h"

class ImageLoader : public WorldLoader_If
{
private:
    QSharedPointer<ImageOptions> lastOptions;
    QVector<ImageParser::Shape> getShapesFromFile(QString filePath, uint64_t colorThreshold);

public:
    virtual bool canLoadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If*> plugins);
    virtual void getUserOptions(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If*> plugins);
    virtual QVector<QString> fileExts() { return QVector<QString>{"Black and White Image (*.png *.jpg *.jpeg *.bmp)"};}
    virtual QVector<WorldObject*> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);
};

#endif
