//! \file
#pragma once

#include "rclcpp/rclcpp.hpp"

#include <Box2D/Box2D.h>
#include <veranda/world_object_component.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

#include "defines.h"

/*!
 * \brief WorldObjectComponent which is just a circular body
 */
class Circle : public WorldObjectComponent
{
    Q_OBJECT

    //! Property: Radius of the circle body
    Property radius = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE,
                               "Radius of the Circle"), QVariant(1.0),
                               &Property::abs_double_validator);

    //! Mapping of the properties specific to circles
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"radius", QSharedPointer<PropertyView>::create(&radius)}
    };

    //! Model containing the circle shape to be drawn
    Model* shape_model = nullptr;

    //! Physics body for the circle
    b2Body* body = nullptr;

    //! Joint connecting the circle to the anchor
    b2WeldJoint* joint = nullptr;

    //! Fixture to make the circle shape in the physics engine
    b2Fixture* fixture = nullptr;

    //! Last box2d world the component was added to
    b2World* _world = nullptr;

    //! Object id of the WorldObject the circle is a part of
    object_id _oid;

public:
    /*!
     * \brief Creates a new circle component
     * \param[in] parent QObject parent of the component
     */
    Circle(QObject* parent=nullptr);

    /*!
     * \brief Creates a copy of the circle
     * \param[in] newParent QObject parent of the clone
     * \return A newly constructed component which is a copy of this one
     */
    WorldObjectComponent* _clone(QObject *newParent);

    /*!
     * \brief Returns the properties specific to the circle component
     * \return The mapping of properties by id/name
     */
    virtual QMap<QString, QSharedPointer<PropertyView>> _getProperties(){
        return _properties;
    }

    /*!
     * \brief Adds the circle component to the box2d world
     * \param[in,out] world The box2d world to add to
     * \param[in] oId object_id of the WorldObject this component is part of
     * \param[in] anchor The anchor body to connect to
     */
    void generateBodies(b2World *world, object_id oId, b2Body *anchor);

    //! Clears the physics bodies created by the last call to generateBodies()
    void clearBodies();

    /*!
     * \brief Getter for the name of the plugin that creates this component
     * \return "org.roboscience.veranda.worldObjectComponent.defaults.circle"
     */
    QString getPluginName() { return CIRCLE_IID; }

private slots:
    //! Creates the circle models
    void _buildModels();

    //! Adds the circle fixture shapes to the physics body
    void _makeFixtures();
};
