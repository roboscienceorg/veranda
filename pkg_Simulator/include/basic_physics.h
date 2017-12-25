#ifndef BASIC_PHYSICS_H
#define BASIC_PHYSICS_H

#include "interfaces/simulator_physics_if.h"
#include <QTimer>

#include <QObject>
#include <QMap>

class BasicPhysics : public Simulator_Physics_If
{
    Q_OBJECT

    struct objectWorldData{
        QVector<b2Body*> staticBodies;
        QVector<b2Body*> dynamicBodies;
        QVector<b2Joint*> joints;

        newWorldObjectPhysics_If *obj;
    };

    b2World *world;

    QMap<object_id, objectWorldData> objects;

    QTimer* tick = nullptr;

    double tickRate;
    double stepTime;


public:
    BasicPhysics(QObject* parent = nullptr);

    bool running(){return tick->isActive();}

public slots:
    virtual void start() override;
    virtual void stop() override;
    virtual void clear() override;

    //Set tick rate and duration
    virtual void setTick(double rate_hz, double duration_s) override;

    //Adds world objects to simulation
    virtual void addWorldObject(newWorldObjectPhysics_If* obj, object_id oId) override;

    //Removes a robot from simulation
    virtual void removeWorldObject(object_id oId) override;

    void step();

signals:
    void worldTick(const b2World*, const double);
};

#endif // BASIC_PHYSICS_H
