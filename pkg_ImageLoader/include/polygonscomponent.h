#ifndef POLYGONSCOMPONENT_H
#define POLYGONSCOMPONENT_H

#include <sdsmt_simulator/world_object_component.h>
#include <Box2D/Box2D.h>

class PolygonsComponent : public WorldObjectComponent
{
    Q_OBJECT

    Model* _polyModel;
    QVector<b2PolygonShape*> _polyShapes;
    QVector<b2Shape*> _shapePtrs;

    b2Body* _polyBody = nullptr;
    QVector<b2Fixture*> _polyFixtures;
    b2World* _world;

    Property _numShapes = Property(PropertyInfo(true, false, PropertyInfo::INT, "Number of polygons in the shape"),
                                   QVariant(0));

    QMap<QString, QSharedPointer<PropertyView>> _properties
    {
        {"polgyon_count", QSharedPointer<PropertyView>(new PropertyView(&_numShapes))}
    };

    b2PolygonShape* copy(b2PolygonShape* orig);

protected:
    QMap<QString, QSharedPointer<PropertyView>> _getProperties(){ return _properties; }

public:
    PolygonsComponent(QVector<b2PolygonShape*> polys, QObject* parent=nullptr);
    ~PolygonsComponent();

    //Constructs copy of component
    WorldObjectComponent *_clone(QObject* newParent=nullptr);

    void generateBodies(b2World* world, object_id oId, b2Body* anchor);
    void clearBodies();
};

#endif
