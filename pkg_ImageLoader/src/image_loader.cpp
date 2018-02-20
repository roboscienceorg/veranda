#include "image_loader.h"
#include "polygonscomponent.h"
#include "imageparser.h"
#include "optiondialog.h"

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
    QImage img(filePath);
    if(img.isNull()) throw std::exception();

    uint64_t fileWidth = img.width(), fileHeight = img.height();

    ImageOptions opt(fileWidth, fileHeight);
    opt.exec();

    uint64_t threshold = opt.getBlackWhiteThreshold();

    QVector<QVector<QPolygonF>> shapes = ImageParser::parseImage(img, threshold);
    QVector<QVector<b2PolygonShape*>> out;

    uint64_t triangleCount = 0;
    for(auto& s : shapes)
        triangleCount += s.size();

    double scalex = opt.getPxPerWidth();
    double scaley = opt.getPxPerHeight();

    out.resize(shapes.size());
    b2Vec2 triBuffer[3];
    for(int i=0; i<shapes.size(); i++)
    {
        QVector<QPolygonF>& shape = shapes[i];
        QVector<b2PolygonShape*>& triangles = out[i];

        for(QPolygonF& poly : shape)
        {
            triBuffer[0].Set(poly[0].x()/scalex, poly[0].y()/scaley);
            triBuffer[1].Set(poly[1].x()/scalex, poly[1].y()/scaley);
            triBuffer[2].Set(poly[2].x()/scalex, poly[2].y()/scaley);

            //Remove 'triangles' that are just a line
            if(std::abs(b2Cross(triBuffer[1] - triBuffer[0], triBuffer[2] - triBuffer[0])) > 0.001)
            {
                b2PolygonShape* triangle = new b2PolygonShape();
                triangle->Set(triBuffer, 3);

                triangles.push_back(triangle);
            }
        }
    }

    return out;
}
