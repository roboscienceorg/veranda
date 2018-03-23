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

#include <memory>

class Polygon : public WorldObjectComponent
{
    Q_OBJECT

    Model* _triangleModel;

    QVector<b2Shape*> _triangleShapes;

    b2Body* _polyBody = nullptr;
    QVector<b2Fixture*> _polyFixtures;
    b2World* _world = nullptr;

    static bool isPoly(const QVariant& v)
    {
        if (!v.canConvert<QVariantList>()) return false;
        QSequentialIterable iterable = v.value<QSequentialIterable>();
        qDebug() << "Loop is list";
        uint64_t count = 0;
        // Can use C++11 range-for:
        for (const QVariant &vi : iterable)
        {
            if(!vi.canConvert<QPointF>()) return false;
            count++;
        }
        qDebug() << count << "points in loop";
        return count >= 3;
    }

    Property _numShapes = Property(PropertyInfo(true, false, PropertyInfo::INT, "Number of polygons in the shape"),
                                   QVariant(0));

    Property _outerShape = Property(PropertyInfo(true, false, PropertyInfo::INT, "The outermost polygon of the shape"),
                                    QVariantList{QPointF(0, 0), QPointF(1, 1), QPointF(1, 0)},
                                    [](const QVariant& _old, const QVariant& _new)
                                    {
                                        qDebug() << "Validate outer loop";
                                        if(isPoly(_new)) return _new;
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

                                        // Can use C++11 range-for:
                                        for (const QVariant &vi : iterable)
                                            if(!isPoly(vi))
                                            {
                                                qDebug() << "Failed";
                                                return _old;
                                            }

                                        return _new;
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
};

#endif // POLYGON_H
