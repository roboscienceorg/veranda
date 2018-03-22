#include "image_loader.h"
#include "imageparser.h"
#include "optiondialog.h"

#include <QDebug>
#include <QDir>

#include <stdexcept>
#include <limits>

QVariant toVariantList(const QPolygonF& poly)
{
    QVariantList list;
    for(const QPointF& p : poly)
        list.append(p);

    return list;
}

QVector<WorldObject *> ImageLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    auto iter = plugins.find("org.sdsmt.sim.2d.worldObjectComponent.defaults.polygon");
    if(iter == plugins.end()) throw std::exception();
    try
    {
        uint64_t colorThreshold = lastOptions->getBlackWhiteThreshold();
        uint64_t crossThreshold = lastOptions->getCrossProductThreshold();
        double scaleY = lastOptions->getPxPerHeight();
        double scaleX = lastOptions->getPxPerWidth();

        qDebug() << "Loading...";
        QVector<ImageParser::Shape> shapes = getShapesFromFile(filePath, colorThreshold);
        QVector<WorldObject*> objects;

        uint64_t objNum = 0;
        for(ImageParser::Shape& sh : shapes)
        {
            if(!sh.outer.size()) continue;

            qDebug() << "Normalize and build World Objects...";
            QPointF max = sh.outer[0], min = max;

            //Normalize each object so it's
            //parts have a reasonable local origin
            for(const QPointF& p : sh.outer)
            {
                min.setX(std::min(min.x(), p.x()));
                max.setX(std::max(max.x(), p.x()));

                min.setY(std::min(min.y(), p.y()));
                max.setY(std::max(max.y(), p.y()));
            }

            for(const QPolygonF& poly : sh.inner)
            {
                for(const QPointF& p : poly)
                {
                    min.setX(std::min(min.x(), p.x()));
                    max.setX(std::max(max.x(), p.x()));

                    min.setY(std::min(min.y(), p.y()));
                    max.setY(std::max(max.y(), p.y()));
                }
            }

            QPointF avg = (min + max) / 2.0;
            for(QPolygonF& poly : sh.inner)
                for(QPointF& p : poly)
                    p = p - avg;

            for(QPointF& p : sh.outer)
                p = p - avg;

            QVariant outer = toVariantList(sh.outer);
            QVariantList inner;
            for(const QPolygonF& poly : sh.inner)
                if(poly.size()) inner.append(toVariantList(poly));

            WorldObjectComponent* comp = iter.value()->createComponent();
            auto props = comp->getProperties();

            props["straightness"]->set(QVariant::fromValue(crossThreshold), true);
            props["scale/horiz"]->set(QVariant::fromValue(scaleX), true);
            props["scale/vert"]->set(QVariant::fromValue(scaleY), true);
            props["outer_shape"]->set(outer, true);
            props["inner_shapes"]->set(inner, true);

            WorldObject* obj(new WorldObject({comp}, "Image Chunk #" + QString::number(objNum++)));
            obj->translate(avg.x()/scaleX, avg.y()/scaleY);
            objects.push_back(obj);
        }
        return objects;
    }catch(std::exception ex){
        qDebug() << "Unable to load image file: " << QDir(filePath).absolutePath();
    }

    return {};
}

bool ImageLoader::canLoadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    QImage img(filePath);
    if(img.isNull()) return false;
    if(!plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.polygon"))
    {
        qDebug() << "Cannot load image files without polygons plugin";
        return false;
    }
    return true;
}

void ImageLoader::getUserOptions(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    QImage img(filePath);
    if(img.isNull()) throw std::exception();
    uint64_t fileWidth = img.width(), fileHeight = img.height();

    lastOptions.reset(new ImageOptions(fileWidth, fileHeight));
    lastOptions->exec();
}

QVector<ImageParser::Shape> ImageLoader::getShapesFromFile(QString filePath, uint64_t colorThreshold)
{
    QImage img(filePath);
    if(img.isNull()) throw std::exception();

    qDebug() << "Parse image...";
    QVector<ImageParser::Shape> shapes = ImageParser::parseImage(img, colorThreshold);

    return shapes;
}
