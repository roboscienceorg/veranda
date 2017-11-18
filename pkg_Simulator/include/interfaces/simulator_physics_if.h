#ifndef SIMULATOR_PHYSICS_H
#define SIMULATOR_PHYSICS_H

#include <QObject>
#include <QVector>

#include <Box2D/Box2D.h>

#include "old_world_object_if.h"

class Simulator_Physics_If : public QObject
{
    Q_OBJECT

public:
    Simulator_Physics_If(QObject* parent = nullptr) : QObject(parent){}

    virtual bool running() = 0;

public slots:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void clear() = 0;

    //Set tick rate and duration
    virtual void setTick(double rate_hz, double duration_s) = 0;

    //Adds world objects to simulation
    virtual void addWorldObject(WorldObjectPhysics_If* obj, object_id oId) = 0;

    //Removes a robot from simulation
    virtual void removeWorldObject(object_id oId) = 0;

signals:
    void physicsStarted();
    void physicsStopped();
    void physicsTickSet(double, double);
};

#endif // SIMULATOR_PHYSICS_H
