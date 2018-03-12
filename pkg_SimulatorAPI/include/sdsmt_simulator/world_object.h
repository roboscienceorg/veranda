#ifndef WORLD_OBJECT_IF_H
#define WORLD_OBJECT_IF_H

#include <QObject>
#include <QString>
#include <QMap>
#include <QVector>

#include "sdsmt_simulator/world_object_component.h"
#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/property.h"

#include <Box2D/Box2D.h>
#include <QVariant>

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
    //UI Interactions
    QMap<QString, QSharedPointer<PropertyView>> _getProperties()
    { return _properties; }

protected slots:
    void _worldTicked(const double t);
    void _syncModels();

public:
    WorldObject(QVector<WorldObjectComponent*> components, QString name = "object", QObject* parent = nullptr);

    //Constructs copy of object
    WorldObject* _clone(QObject* newParent=nullptr);

    QVector<WorldObjectComponent*> getComponents()
    {return _components; }

    bool usesChannels()
    { return _useChannels; }

    //Physics Interactions
    void generateBodies(b2World* world, object_id oId, b2Body* anchorBody = nullptr);

    void clearBodies();

    //ROS Interactions
    void setROSNode(std::shared_ptr<rclcpp::Node> node);

public slots:
    void connectChannels();
    void disconnectChannels();
};

#endif // WORLD_OBJECT_IF_H
