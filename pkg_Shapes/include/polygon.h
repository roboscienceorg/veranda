#ifndef POLYGON_H
#define POLYGON_H

#include "rclcpp/rclcpp.hpp"

#include <Box2D/Box2D.h>
#include <sdsmt_simulator/world_object_component.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>
#include <QVariantList>
#include <QVariant>

#include <memory>

#include "defines.h"

class Polygon : public WorldObjectComponent
{
    Q_OBJECT

    Model* _triangleModel;

    QVector<b2Shape*> _triangleShapes;

    b2Body* _polyBody = nullptr;
    QVector<b2Fixture*> _polyFixtures;
    b2World* _world = nullptr;

    static QVariant toPoly(const QVariant& v)
    {
        QVariantList poly;
        QSequentialIterable iterable = v.value<QSequentialIterable>();
        for (const QVariant &vi : iterable)
        {
            if(vi.canConvert<QPointF>())
            {
                QPointF p = vi.value<QPointF>();
                poly.append(QVariant(QVariantList{p.x(), p.y()}));
            }
            else
            {
                QVariantList pnt;
                QSequentialIterable iterable2 = vi.value<QSequentialIterable>();
                for(const QVariant& v : iterable2)
                {
                    pnt.append(v.toDouble());
                    if(pnt.size() == 2) break;
                }
                poly.append(QVariant(pnt));
            }
        }
        return poly;
    }

    static bool isPoly(const QVariant& v)
    {
        if (!v.canConvert<QVariantList>()) return false;
        QSequentialIterable iterable = v.value<QSequentialIterable>();
        uint64_t count = 0;
        // Can use C++11 range-for:
        for (const QVariant &vi : iterable)
        {
            bool good = false;
            if(vi.canConvert<QPointF>()) good = true;
            else if(vi.canConvert<QVariantList>())
            {
                QSequentialIterable iterable2 = vi.value<QSequentialIterable>();
                if(iterable2.size() == 2)
                {
                    good = true;
                    for(const QVariant& vii : iterable2)
                        if(!vii.canConvert<double>())
                        {
                            good = false;
                            break;
                        }
                }
            }
            if(!good) return false;

            count++;
        }
        return count >= 3;
    }

    Property _numShapes = Property(PropertyInfo(true, false, PropertyInfo::INT, "Number of polygons in the shape"),
                                   QVariant(0));

    Property _outerShape = Property(PropertyInfo(true, false, PropertyInfo::INT, "The outermost polygon of the shape"),
                                    QVariantList{QVariantList{0, 0}, QVariantList{1, 1}, QVariantList{1, 0}},
                                    [](const QVariant& _old, const QVariant& _new)
                                    {
                                        qDebug() << "Validate outer loop";
                                        if(isPoly(_new)) return toPoly(_new);
                                        qDebug() << "Failed";
                                        return _old;
                                    });

    Property _innerShapes = Property(PropertyInfo(true, false, PropertyInfo::INT, "Inner polygons defining holes"),
                                   QVariantList(),
                                    [](const QVariant& _old, const QVariant& _new)
                                    {
                                        qDebug() << "Validate inner loops";
                                        if (!_new.canConvert<QVariantList>()) return _old;
                                        qDebug() << "Is a list";
                                        QSequentialIterable iterable = _new.value<QSequentialIterable>();

                                        QVariantList polyList;
                                        // Can use C++11 range-for:
                                        for (const QVariant &vi : iterable)
                                        {
                                            if(!isPoly(vi))
                                            {
                                                qDebug() << "Failed";
                                                return _old;
                                            }
                                            polyList.append(toPoly(vi));
                                        }

                                        return QVariant(polyList);
                                    });

    Property _fullDraw = Property(PropertyInfo(true, false, PropertyInfo::BOOL, "Whether or not to draw all triangles"),
                                  QVariant(false), &Property::bool_validator);

    Property _straight = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE, "How close to straight an edge has to be to have points removed"),
                                  QVariant(0), &Property::abs_double_validator);

    Property _scalex = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE, "Horizontal scaling factor"),
                                  QVariant(1), &Property::abs_double_validator);

    Property _scaley = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE, "Vertical scaling factor"),
                                  QVariant(1), &Property::abs_double_validator);

#define prop(x) QSharedPointer<PropertyView>(new PropertyView(&(x)))
    QMap<QString, QSharedPointer<PropertyView>> _properties
    {
        {"polgyon_count", prop(_numShapes)},
        {"outer_shape", prop(_outerShape)},
        {"inner_shapes", prop(_innerShapes)},
        //{"draw_triangles", prop(_fullDraw)},
        {"straightness", prop(_straight)},
        {"scale/horiz", prop(_scalex)},
        {"scale/vert", prop(_scaley)}
    };
#undef prop

    b2PolygonShape* copy(b2PolygonShape* orig);
    void makeTriangles();
    void makeFixtures();

protected:
    QMap<QString, QSharedPointer<PropertyView>> _getProperties(){ return _properties; }

public:
    Polygon(QObject* parent=nullptr);
    ~Polygon();

    //Constructs copy of component
    WorldObjectComponent *_clone(QObject* newParent=nullptr);

    void generateBodies(b2World* world, object_id oId, b2Body* anchor);
    void clearBodies();

    QString getPluginName(){ return POLYGON_IID; }
};

#endif // POLYGON_H
