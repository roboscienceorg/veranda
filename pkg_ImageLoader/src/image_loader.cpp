#include "image_loader.h"
#include "polygonscomponent.h"
#include "imageparser.h"

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

            qDeleteAll(poly);
        }

        return objects;
    }catch(std::exception ex){
        qDebug() << "Unable to load image file: " << QDir(filePath).absolutePath();
    }

    return {};
}

QVector<QVector<b2PolygonShape*>> ImageLoader::getShapesFromFile(QString filePath)
{
    QVector<QVector<QPolygonF>> shapes = ImageParser::parseImage(filePath);
    QVector<QVector<b2PolygonShape*>> out;

    out.resize(shapes.size());
    b2Vec2 triBuffer[3];
    for(int i=0; i<shapes.size(); i++)
    {
        QVector<QPolygonF>& shape = shapes[i];
        QVector<b2PolygonShape*>& triangles = out[i];

        for(QPolygonF& poly : shape)
        {
            triBuffer[0].Set(poly[0].x(), poly[0].y());
            triBuffer[1].Set(poly[1].x(), poly[1].y());
            triBuffer[2].Set(poly[2].x(), poly[2].y());

            b2PolygonShape* triangle = new b2PolygonShape();
            triangle->Set(triBuffer, 3);

            triangles.push_back(triangle);
        }
    }

    return out;
}
