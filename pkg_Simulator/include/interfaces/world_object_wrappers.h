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

    WorldObjectComponent* _comp = nullptr;

public:
    WorldObjectProperties(WorldObjectComponent* comp, QObject* parent=nullptr) : QObject(parent), _comp(comp){}

    //Drawing Interactions
    QVector<Model*> getModels()
    { return _comp->getModels(); }

    QString getName()
    { return ""; }

    QString getType()
    { return ""; }

    //UI Interactions
    QMap<QString, QSharedPointer<PropertyView>> getProperties()
    { return _comp->getProperties(); }

    bool usesChannels()
    { return _comp->usesChannels(); }

    WorldObject* getObject()
    { return dynamic_cast<WorldObject*>(_comp); }

    WorldObjectComponent* getComponent()
    { return _comp; }

public slots:
    void connectChannels()
    { _comp->connectChannels(); }

    void disconnectChannels()
    { _comp->disconnectChannels(); }
};

class WorldObjectPhysics : public QObject
{
    Q_OBJECT

    WorldObject* _obj;

public:
    WorldObjectPhysics(WorldObject* obj, QObject* parent=nullptr) : QObject(parent), _obj(obj){}

    virtual void generateBodies(b2World* world, object_id oId){_obj->generateBodies(world, oId);}
    virtual void clearBodies(){_obj->clearBodies();}

public slots:
    //Interface to update on world ticks
    virtual void syncModels(){_obj->syncModels();}
    virtual void worldTicked(const double t){_obj->worldTicked(t);}
};
#endif // WORLD_OBJECT_WRAPPERS_H
