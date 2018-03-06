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
    constexpr static double PI = 3.14159265359;
    constexpr static double RAD2DEG = 360.0/(2*PI);
    constexpr static double DEG2RAD = 1.0/RAD2DEG;

    WorldObjectComponent_If(QObject* parent=nullptr) : QObject(parent){}

    //Constructs copy of component
    virtual WorldObjectComponent_If* clone(QObject* newParent=nullptr) = 0;

    //Interactions with ROS
    virtual void setROSNode(std::shared_ptr<rclcpp::Node> node) = 0;

    //Drawing Interactions
    virtual QVector<Model*> getModels() = 0;

    //UI Interactions
    virtual QMap<QString, PropertyView>& getProperties() = 0;
    virtual QString getPropertyGroup() = 0;

    virtual bool usesChannels() = 0;

    virtual QVector<b2Body*> generateBodies(b2World* world, object_id oId, b2Body* anchor) = 0;
    virtual void clearBodies(b2World* world) = 0;

    static void moveBodyToLocalSpaceOfOtherBody(b2Body* bodyToMove, b2Body* bodyToReference,
                                                double xRelative, double yRelative, double thetaRelativeDegrees);

public slots:
    virtual void connectChannels() = 0;
    virtual void disconnectChannels() = 0;
    virtual void worldTicked(const b2World* w, const double t) = 0;
    virtual void setObjectMass(double mass) = 0;

signals:
    virtual void massChanged(WorldObjectComponent_If* component, double mass);
};


inline void WorldObjectComponent_If::moveBodyToLocalSpaceOfOtherBody(b2Body* bodyToMove, b2Body* bodyToReference,
                                                                     double xRelative=0, double yRelative=0, double thetaRelativeDegrees=0)
{
    double referenceAngle = bodyToReference->GetAngle();
    double cosT = cos(referenceAngle);
    double sinT = sin(referenceAngle);

    b2Vec2 relativeLoc(xRelative, yRelative);

    //Apply rotation matrix
    b2Vec2 newLoc;
    newLoc.x = cosT * relativeLoc.x - sinT * relativeLoc.y;
    newLoc.y = sinT * relativeLoc.x + cosT * relativeLoc.y;

    //Offset
    newLoc += b2Vec2(bodyToReference->GetWorldCenter().x, bodyToReference->GetWorldCenter().y);

    bodyToMove->SetTransform(newLoc, thetaRelativeDegrees*DEG2RAD + referenceAngle);
}

#endif // WORLD_OBJECT_COMPONENT_H
