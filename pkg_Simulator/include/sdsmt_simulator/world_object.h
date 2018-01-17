#ifndef WORLD_OBJECT_IF_H
#define WORLD_OBJECT_IF_H

#include <QObject>
#include <QString>
#include <QMap>
#include <QVector>

#include "sdsmt_simulator/world_object_component_if.h"
#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/property.h"

#include <Box2D/Box2D.h>
#include <QVariant>

class WorldObject : public QObject
{
    Q_OBJECT

    constexpr static double PI = 3.14159265359;
    constexpr static double RAD2DEG = 360.0/(2*PI);
    constexpr static double DEG2RAD = 1.0/RAD2DEG;

    void _transformToStart(b2Body* body);

    QVector<WorldObjectComponent_If*> _components;

    bool _useChannels = false;

    Property _objName;
    Property _locX = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "X coord of the object"),
                                QVariant(0.0), &Property::double_validator);

    Property _locY = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Y coord of the object"),
                                QVariant(0.0), &Property::double_validator);

    Property _locTheta = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Angle of the object"),
                                QVariant(0.0), &Property::angle_validator);

    Model* debugModel = nullptr;
    b2Body* anchorBody = nullptr;

    QVector<Model*> _models;
    QMap<QString, PropertyView> _properties
    {
        {"name", &_objName},
        {"X", &_locX},
        {"Y", &_locY},
        {"Theta", &_locTheta}
    };

public:
    WorldObject(QVector<WorldObjectComponent_If*> components, QObject* parent = nullptr);

    //Constructs copy of object
    WorldObject* clone(QObject* newParent=nullptr);

    QVector<WorldObjectComponent_If*> getComponents()
    {return _components; }

    //Drawing Interactions
    QVector<Model*> getModels()
    { return _models; }

    //UI Interactions
    QMap<QString, PropertyView>& getProperties()
    { return _properties; }

    bool usesChannels()
    { return _useChannels; }

    //Physics Interactions
    void generateBodies(b2World* world, object_id oId);

    void clearBodies(b2World* world);

    //ROS Interactions
    void setROSNode(std::shared_ptr<rclcpp::Node> node);

public slots:
    void connectChannels();
    void disconnectChannels();
    void worldTicked(const b2World* w, const double t);
};

class WorldObjectProperties : public QObject
{
    Q_OBJECT

    WorldObject* _obj;
public:
    WorldObjectProperties(WorldObject* obj, QObject* parent=nullptr) : QObject(parent), _obj(obj){}

    //Drawing Interactions
    QVector<Model*> getModels()
    { return _obj->getModels(); }

    //UI Interactions
    QMap<QString, PropertyView>& getProperties()
    { return _obj->getProperties(); }

    bool usesChannels()
    { return _obj->usesChannels(); }

public slots:
    void connectChannels()
    { _obj->connectChannels(); }

    void disconnectChannels()
    { _obj->disconnectChannels(); }
};

class WorldObjectPhysics : public QObject
{
    Q_OBJECT

    WorldObject* _obj;

public:
    WorldObjectPhysics(WorldObject* obj, QObject* parent=nullptr) : QObject(parent), _obj(obj){}

    virtual void generateBodies(b2World* world, object_id oId){_obj->generateBodies(world, oId);}
    virtual void clearDynamicBodies(){}
    virtual void clearStaticBodies(){}

public slots:
    //Interface to update on world ticks
    virtual void worldTicked(const b2World* w, const double t){_obj->worldTicked(w, t);}
};
#endif // WORLD_OBJECT_IF_H
