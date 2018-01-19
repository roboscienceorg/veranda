#ifndef WORLD_OBJECT_COMPONENT_H
#define WORLD_OBJECT_COMPONENT_H

#include <QObject>
#include <QString>
#include <QMap>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/property.h"
#include "dllapi.h"

class SDSMT_SIMULATOR_API WorldObjectComponent_If : public QObject
{
    Q_OBJECT

public:
    WorldObjectComponent_If(QObject* parent=nullptr) : QObject(parent){}

    //Constructs copy of component
    virtual WorldObjectComponent_If* clone(QObject* newParent=nullptr) = 0;

    //Interactions with ROS
    virtual void setROSNode(std::shared_ptr<rclcpp::Node> node) = 0;

    //Drawing Interactions
    virtual QVector<Model*> getModels() = 0;

    //UI Interactions
    virtual QMap<QString, PropertyView> getProperties() = 0;
    virtual QString getPropertyGroup() = 0;

    virtual bool usesChannels() = 0;

    virtual QVector<b2Body*> generateBodies(b2World* world, object_id oId, b2Body* anchor) = 0;
    virtual void clearBodies(b2World* world) = 0;

public slots:
    virtual void connectChannels() = 0;
    virtual void disconnectChannels() = 0;
    virtual void worldTicked(const b2World* w, const double t) = 0;
};

#endif // WORLD_OBJECT_COMPONENT_H
