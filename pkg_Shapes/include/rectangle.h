#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "rclcpp/rclcpp.hpp"

#include <Box2D/Box2D.h>
#include <sdsmt_simulator/world_object_component.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

class Rectangle : public WorldObjectComponent
{
    Q_OBJECT

    Property height = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                               "Height of the rectangle"), QVariant(1.0),
                               &Property::abs_double_validator);

    Property width = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                               "Width of the rectangle"), QVariant(1.0),
                               &Property::abs_double_validator);

    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"width", QSharedPointer<PropertyView>::create(&width)},
        {"heigh", QSharedPointer<PropertyView>::create(&height)}
    };

    Model* shape_model = nullptr;

    b2Body* body = nullptr;
    b2WeldJoint* joint = nullptr;
    b2Fixture* fixture;

    b2World* _world = nullptr;

    object_id _oid;

public:
    Rectangle(QObject* parent=nullptr);

    WorldObjectComponent* clone(QObject *newParent);

    virtual QMap<QString, QSharedPointer<PropertyView>> _getProperties(){
        return _properties;
    }

    void generateBodies(b2World *world, object_id oId, b2Body *anchor);
    void clearBodies();

private slots:
    void _buildModels();
    void _makeFixtures();
};

#endif // FLOATER_DRIVETRAIN_H
