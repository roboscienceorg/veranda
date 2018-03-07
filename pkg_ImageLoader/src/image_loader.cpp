#include "image_loader.h"
#include "polygonscomponent.h"
#include "imageparser.h"
#include "optiondialog.h"

#include <QDebug>
#include <QDir>

#include <stdexcept>
#include <limits>

QVector<QSharedPointer<WorldObject> > ImageLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    try
    {
        QVector<QVector<b2PolygonShape*>> shapes = getShapesFromFile(filePath);
        QVector<QSharedPointer<WorldObject>> objects;

        for(QVector<b2PolygonShape*>& poly : shapes)
        {
            if(!poly.size()) continue;

            //Normalize each object so it's
            //parts have a reasonable local origin
            b2Vec2 max = poly[0]->m_vertices[0], min = max;
            for(b2PolygonShape* s : poly)
                for(b2Vec2* it = s->m_vertices; it < s->m_vertices + s->m_count; it++)
                {
                    min = b2Min(min, *it);
                    max = b2Max(max, *it);
                }

            b2Vec2 avg = (min + max) / 2.0;
            for(b2PolygonShape* s : poly)
                for(b2Vec2* it = s->m_vertices; it < s->m_vertices + s->m_count; it++)
                    *it = (*it) - avg;

            PolygonsComponent* comp = new PolygonsComponent(poly);
            QSharedPointer<WorldObject> obj(new WorldObject({comp}));
            obj->translate(avg.x, avg.y);
            objects.push_back(obj);

            qDeleteAll(poly);
        }

        return objects;
    }catch(std::exception ex){
        qDebug() << "Unable to load image file: " << QDir(filePath).absolutePath();
    }

    return {};
}

bool ImageLoader::canLoadFile(QString filePath)
{
    QImage img(filePath);
    if(img.isNull()) return false;
    return true;
}

void ImageLoader::getUserOptions(QString filePath)
{
    QImage img(filePath);
    if(img.isNull()) throw std::exception();
    uint64_t fileWidth = img.width(), fileHeight = img.height();

    lastOptions.reset(new ImageOptions(fileWidth, fileHeight));
    lastOptions->exec();
}

QVector<QVector<b2PolygonShape*>> ImageLoader::getShapesFromFile(QString filePath)
{
    QImage img(filePath);
    if(img.isNull()) throw std::exception();

    uint64_t threshold = lastOptions->getBlackWhiteThreshold();

    QVector<QVector<QPolygonF>> shapes = ImageParser::parseImage(img, threshold);
    QVector<QVector<b2PolygonShape*>> out;

    uint64_t triangleCount = 0;
    for(auto& s : shapes)
        triangleCount += s.size();

    double scalex = lastOptions->getPxPerWidth();
    double scaley = lastOptions->getPxPerHeight();

    b2Vec2 triBuffer[3];
    for(int i=0; i<shapes.size(); i++)
    {
        QVector<QPolygonF>& shape = shapes[i];
        QVector<b2PolygonShape*>triangles;

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
        if(triangles.size()) out.push_back(triangles);
    }

    return out;
}
