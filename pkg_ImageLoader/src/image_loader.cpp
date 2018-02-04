#include "image_loader.h"
#include <QDebug>

#include <stdexcept>

QVector<WorldObject*> ImageLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    try
    {
        QVector<QVector<b2Shape*>> shapes = getShapesFromFile(filePath);
    }catch(std::exception ex){qDebug() << "Unable to load image file: " << filePath;}

    return {};
}

QVector<QVector<b2Shape*>> ImageLoader::getShapesFromFile(QString filePath)
{

}
