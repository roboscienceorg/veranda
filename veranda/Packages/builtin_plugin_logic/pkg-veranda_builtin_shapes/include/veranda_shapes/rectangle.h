//! \file
#pragma once

#include "rclcpp/rclcpp.hpp"

#include <Box2D/Box2D.h>
#include <veranda_core/world_object_component.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

/*!
 * \brief WorldObjectComponent which is a rectangular body
 */
class Rectangle : public WorldObjectComponent
{
    Q_OBJECT

    //! Property: Height of the rectangle body
    Property height = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE,
                               "Height of the rectangle"), QVariant(1.0),
                               &Property::abs_double_validator);

    //! Property: Width of the rectangle body
    Property width = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE,
                               "Width of the rectangle"), QVariant(1.0),
                               &Property::abs_double_validator);

    //! Mapping of the properties specific to rectangles
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"width", QSharedPointer<PropertyView>::create(&width)},
        {"height", QSharedPointer<PropertyView>::create(&height)}
    };

    //! Model containing the rectangle shape to be drawn
    Model* shape_model = nullptr;

    //! Physics body for the rectangle
    b2Body* body = nullptr;

    //! Joint connecting the rectangle to the anchor
    b2WeldJoint* joint = nullptr;

    //! Fixture to make the rectangle shape in the physics engine
    b2Fixture* fixture = nullptr;

    //! Last box2d world the component was added to
    b2World* _world = nullptr;

    //! Object id of the WorldObject the circle is a part of
    object_id _oid;

    //! IID of plugin that generated this object
    const QString _pluginIID;

public:
    /*!
     * \brief Creates a new rectangle component
     * \param[in] parent QObject parent of the component
     */
    Rectangle(const QString& pluginIID, QObject* parent=nullptr);
    virtual ~Rectangle() override {}

    /*!
     * \brief Creates a copy of the rectangle
     * \param[in] newParent QObject parent of the clone
     * \return A newly constructed component which is a copy of this one
     */
    WorldObjectComponent* _clone(QObject *newParent) override;

    /*!
     * \brief Returns the properties specific to the rectangle component
     * \return The mapping of properties by id/name
     */
    virtual QMap<QString, QSharedPointer<PropertyView>> _getProperties() override {
        return _properties;
    }

    /*!
     * \brief Adds the rectangle component to the box2d world
     * \param[in,out] world The box2d world to add to
     * \param[in] oId object_id of the WorldObject this component is part of
     * \param[in] anchor The anchor body to connect to
     */
    void _generateBodies(b2World *world, object_id oId, b2Body *anchor) override;

    //! Clears the physics bodies created by the last call to generateBodies()
    void _clearBodies() override;

    /*!
     * \brief Returns the plugin IID
     * \return The IID of the plugin providing this component
     */
    QString getPluginName() override { return _pluginIID; }

private slots:
    //! Creates the rectangle models
    void _buildModels();

    //! Adds the rectangle fixture shapes to the physics body
    void _makeFixtures();
};\
