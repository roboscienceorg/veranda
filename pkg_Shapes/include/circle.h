#ifndef CIRCLE_H
#define CIRCLE_H

#include "rclcpp/rclcpp.hpp"

#include <Box2D/Box2D.h>
#include <sdsmt_simulator/world_object_component.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

class Circle : public WorldObjectComponent
{
    Q_OBJECT

    Property radius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                               "Width of the Circle"), QVariant(1.0),
                               &Property::abs_double_validator);

    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"radius", QSharedPointer<PropertyView>::create(&radius)}
    };

    Model* shape_model = nullptr;

    b2Body* body = nullptr;
    b2WeldJoint* joint = nullptr;
    b2Fixture* fixture = nullptr;

    b2World* _world = nullptr;
    object_id _oid;

public:
    Circle(QObject* parent=nullptr);

    WorldObjectComponent* _clone(QObject *newParent);

    virtual QMap<QString, QSharedPointer<PropertyView>> _getProperties(){
        return _properties;
    }

    void generateBodies(b2World *world, object_id oId, b2Body *anchor);
    void clearBodies();

private slots:
    void _buildModels();
    void _makeFixtures();
};

#endif
