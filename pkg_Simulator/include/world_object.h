#ifndef WORLD_OBJECT_IF_H
#define WORLD_OBJECT_IF_H

#include <QObject>
#include <QString>
#include <QMap>
#include <QVector>

#include "sdsmt_simulator/world_object_component_if.h"
#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/property.h"

#include "Box2D/Box2D.h"

class WorldObject : public QObject
{
    Q_OBJECT

    QVector<WorldObjectComponent_If*> _components;

    bool _useChannels = false;

    Property _objName;

    Model* debugModel = nullptr;
    b2Body* anchorBody = nullptr;

    QVector<Model*> _models;
    QMap<QString, PropertyView> _properties
    {
        {"name", &_objName}
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
    QMap<QString, PropertyView> getProperties()
    { return _properties; }

    bool usesChannels()
    { return _useChannels; }

    //Physics Interactions
    void generateBodies(b2World* world, object_id oId);

    void clearBodies(b2World* world);

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
    QMap<QString, PropertyView> getProperties()
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
