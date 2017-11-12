#ifndef WORLD_OBJECT_IF_H
#define WORLD_OBJECT_IF_H

#include <QObject>
#include <QVector>

#include <Box2D/Box2D.h>

#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/properties_if.h"

class WorldObject_If : public QObject
{
    Q_OBJECT

public:
    WorldObject_If(QObject* parent=nullptr) : QObject(parent){}

    //Virtual clone
    virtual WorldObject_If* clone(QObject* newParent=nullptr) = 0;

    //Interfaces for UI to display properties
    virtual QMap<QString, PropertyView>& getAllProperties() = 0;
    virtual QString propertyGroupName() = 0;

    //Interface for world view to draw
    virtual QVector<Model*> getModels(){ return {}; }

    //Tells the physics engine how many bodies are needed
    virtual uint64_t staticBodiesRequired() { return 0; }
    virtual uint64_t dynamicBodiesRequired() { return 0; }

    //Physics engine assigns bodies
    virtual void setStaticBodies(QVector<b2Body*>&){}
    virtual QVector<b2JointDef*> setDynamicBodies(QVector<b2Body*>&){ return {}; }
    virtual void clearStaticBodies(){}
    virtual void clearDynamicBodies(){}

public slots:
    //Interface to connect/disconnect to external communications
    virtual void connectChannels(){}
    virtual void disconnectChannels(){}

    //Interface to update on world ticks
    virtual void worldTicked(const b2World*, const double&){}
};

class WorldObjectProperties_If : public QObject
{
    Q_OBJECT
    WorldObject_If* _obj;

public:
    WorldObjectProperties_If(WorldObject_If* obj, QObject* parent=nullptr) : QObject(parent), _obj(obj){}

    //Interfaces for UI to display properties
    virtual QMap<QString, PropertyView>& getAllProperties(){return _obj->getAllProperties();}
    virtual QString propertyGroupName(){return _obj->propertyGroupName();}

    //Interface for world view to draw
    virtual QVector<Model*> getModels(){return _obj->getModels();}

    //Interface to connect/disconnect to external communications
    virtual void connectChannels(){_obj->connectChannels();}
    virtual void disconnectChannels(){_obj->disconnectChannels();}
};

class WorldObjectPhysics_If : public QObject
{
    Q_OBJECT

    WorldObject_If* _obj;

public:
    WorldObjectPhysics_If(WorldObject_If* obj, QObject* parent=nullptr) : QObject(parent), _obj(obj){}

    //Tells the physics engine how many bodies are needed
    uint64_t staticBodiesRequired() { return _obj->staticBodiesRequired(); }
    uint64_t dynamicBodiesRequired() { return _obj->dynamicBodiesRequired(); }

    //Physics engine assigns bodies
    void setStaticBodies(QVector<b2Body*> v){_obj->setStaticBodies(v);}
    QVector<b2JointDef*>  setDynamicBodies(QVector<b2Body*> v){return _obj->setDynamicBodies(v);}

    virtual void clearDynamicBodies(){_obj->clearDynamicBodies();}
    virtual void clearStaticBodies(){_obj->clearStaticBodies();}

public slots:
    //Interface to update on world ticks
    virtual void worldTicked(const b2World* w, const double t){_obj->worldTicked(w, t);}
};

#endif // WORLD_OBJECT_IF_H
