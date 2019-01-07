//! \file
#pragma once

#include <QObject>
#include <QString>
#include <QMap>
#include <QVector>
#include <QJsonArray>
#include <QJsonObject>

#include "veranda_core/api/world_object_component.h"
#include "veranda_core/api/model.h"
#include "veranda_core/api/property.h"

#include "Box2D/Box2D.h"
#include <QVariant>

/*!
 * \brief Container class for a group of world object components
 * The WorldObject is the main element of a simulation. Multiple WorldObjects
 * can be placed in a simulation and they will interact with each other. Each WorldObject
 * is made by combining any number of WorldObjectComponents and jointing them together.
 * The WorldObject itself is a WorldObjectComponent, so it could be used as a sub-object
 * of a different WorldObject.
 *
 * Just like a WorldObjectComponent, a WorldObject has Properties, Models, and Bodies.
 * The Properties of a WorldObject contain the standard ones in all WorldObjectComponents joined
 * with all of the properties of the children components. The properties of the of a child are prepended
 * with the child's component name (enumerated if there are duplicates). The Models of a WorldObject
 * will be all of the models of its children, and possibly an extra debugging model. The WorldObject
 * will create 1 core body which all of the component children will joint themselves to
 *
 * \todo Move most of this functionality into WorldObjectComponent
 */
class VERANDA_CORE_API_DLL WorldObject : public WorldObjectComponent
{
    Q_OBJECT

    //! Children components of the object
    QVector<WorldObjectComponent*> _components;

    //! OR of usesChannels for all child components
    bool _useChannels = false;

    //! Model that can be drawn to visualize the WorldObject's location
    Model* _debugModel = nullptr;

    //! Central body that components can anchor to
    b2Body* _anchorBody = nullptr;

    //! The b2World last set by generateBodies
    b2World* _world = nullptr;

    //! Union of all properties provided by the object's components
    QMap<QString, QSharedPointer<PropertyView>> _properties;

    //! List of properties specific only to self and not children
    QMap<QString, QSharedPointer<PropertyView>> _selfProperties;

protected:
    QMap<QString, QSharedPointer<PropertyView>> _getProperties() override
    { return _properties; }

public:
    /*!
     * \brief Constructs a WorldObject from a set of components.
     * The children components will becomed owned by the WorldObject; it will
     * delete them when it is destroyed.
     * \param[in] components Children components of the world object
     * \param[in] name The name of the object (Default "object")
     * \param[in] type The type of the object (Default "Component Group")
     * \param[in] parent QObject parent
     */
    WorldObject(QVector<WorldObjectComponent*> components, QString name = "object", QString type = "Component Group", QObject* parent = nullptr);
    virtual ~WorldObject() override {}

    //Constructs copy of object
    WorldObject* _clone(QObject* newParent=nullptr) override;

    /*!
     * \brief Getter for the children components
     * \return A vector of the children component pointers
     */
    QVector<WorldObjectComponent*> getComponents()
    {return _components; }

    bool usesChannels() override
    { return _useChannels; }

    //Physics Interactions
    void _generateBodies(b2World* world, object_id oId, b2Body* anchorBody = nullptr) override;

    void _clearBodies() override;

    QString getPluginName() override { return ""; }

    QMap<QString, QSharedPointer<PropertyView>> getSelfProperties(){ return _selfProperties; }
};
