//! \file
#pragma once

#include <QObject>
#include <QString>
#include <QMap>
#include <QVector>
#include <QJsonArray>
#include <QJsonObject>

#include "sdsmt_simulator/world_object_component.h"
#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/property.h"

#include <Box2D/Box2D.h>
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
class SDSMT_SIMULATOR_API WorldObject : public WorldObjectComponent
{
    Q_OBJECT

    QVector<WorldObjectComponent*> _components;

    bool _useChannels = false;

    Model* _debugModel = nullptr;
    b2Body* _anchorBody = nullptr;
    b2World* _world = nullptr;

    QMap<QString, QSharedPointer<PropertyView>> _properties;

protected:
    QMap<QString, QSharedPointer<PropertyView>> _getProperties()
    { return _properties; }

protected slots:
    void _worldTicked(const double t);
    void _syncModels();

public:
    /*!
     * \brief Constructs a WorldObject from a set of components.
     * The children components will becomed owned by the WorldObject; it will
     * delete them when it is destroyed.
     * \param components Children components of the world object
     * \param name The name of the object (Default "object")
     * \param parent QObject parent
     */
    WorldObject(QVector<WorldObjectComponent*> components, QString name = "object", QObject* parent = nullptr);

    //Constructs copy of object
    WorldObject* _clone(QObject* newParent=nullptr);

    /*!
     * \brief Getter for the children components
     * \return A vector of the children component pointers
     */
    QVector<WorldObjectComponent*> getComponents()
    {return _components; }

    bool usesChannels()
    { return _useChannels; }

    //Physics Interactions
    void generateBodies(b2World* world, object_id oId, b2Body* anchorBody = nullptr);

    void clearBodies();

    //ROS Interactions
    void setROSNode(std::shared_ptr<rclcpp::Node> node);

    //WILL BE USED TO OUTPUT ADDITIONAL INFO
    //void readJson(const QJsonObject &json);
    //void writeJson(QJsonObject &json) const;

    QString getPluginName() { return ""; }

public slots:
    void connectChannels();
    void disconnectChannels();
};
