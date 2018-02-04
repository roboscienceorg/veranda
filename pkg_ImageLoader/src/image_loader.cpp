#include "image_loader.h"
#include "polygonscomponent.h"

#include <QDebug>
#include <QDir>

#include <stdexcept>

QVector<WorldObject*> ImageLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    try
    {
        QVector<QVector<b2PolygonShape*>> shapes = getShapesFromFile(filePath);
        QVector<WorldObject*> objects;

        for(QVector<b2PolygonShape*>& poly : shapes)
        {
            PolygonsComponent* comp = new PolygonsComponent(poly);
            objects.push_back(new WorldObject({comp}));
        }

        return objects;
    }catch(std::exception ex){
        qDebug() << "Unable to load image file: " << QDir(filePath).absolutePath();
    }

    return {};
}

QVector<QVector<b2PolygonShape*>> ImageLoader::getShapesFromFile(QString filePath)
{
    throw std::exception();
}
