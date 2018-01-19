#ifndef WORLD_OBJECT_WRAPPERS_H
#define WORLD_OBJECT_WRAPPERS_H

#include <sdsmt_simulator/world_object.h>
#include <sdsmt_simulator/property.h>
#include <sdsmt_simulator/model.h>
#include <Box2D/Box2D.h>

#include <QObject>
#include <QMap>


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
#endif // WORLD_OBJECT_WRAPPERS_H
