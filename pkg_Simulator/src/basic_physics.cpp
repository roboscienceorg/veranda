#include "basic_physics.h"
#include <QDebug>

#include <string>
#include <stdexcept>

BasicPhysics::BasicPhysics(QObject *parent) : Simulator_Physics_If(parent)
{
    //No gravity for top down simulation
    b2Vec2 gravity(0.0f, 0.0f);
    //Create world
    world = new b2World(gravity);

    tickRate = 1;
    stepTime = 1;

    tick = new QTimer(this);
    connect(tick, &QTimer::timeout, this, &BasicPhysics::step);
}

void BasicPhysics::start()
{
    tick->setInterval(1000.0f / tickRate);
    tick->setTimerType(Qt::PreciseTimer);

    tick->start();

    emit physicsStarted();
}

void BasicPhysics::stop()
{
    tick->stop();

    emit physicsStopped();
}

void BasicPhysics::clear()
{
    delete world;
    b2Vec2 gravity(0.0f, 0.0f);
    world = new b2World(gravity);
}

void BasicPhysics::setTick(double rate_hz, double duration_s)
{
    tickRate = rate_hz;
    stepTime = duration_s;

    //TODO :: Update QTimer with new rate settings

    emit physicsTickSet(tickRate, stepTime);
}

void BasicPhysics::addWorldObject(WorldObjectPhysics_If* obj, object_id oId)
{
    if(objects.contains(oId)) throw std::logic_error("object with id " + std::to_string(oId) + " already exists");

    objectWorldData& worldDat = objects[oId];
    worldDat.obj = obj;

    connect(this, &BasicPhysics::worldTick, obj, &WorldObjectPhysics_If::worldTicked);

    for(int i=0; i<obj->staticBodiesRequired(); i++)
    {
        b2BodyDef robotBodyDef;
        robotBodyDef.type = b2_staticBody;
        robotBodyDef.position.Set(0,0);

        b2Body* body = world->CreateBody(&robotBodyDef);
        worldDat.staticBodies.push_back(body);
    }

    for(int i=0; i<obj->dynamicBodiesRequired();i++)
    {
        b2BodyDef robotBodyDef;
        robotBodyDef.type = b2_dynamicBody;
        robotBodyDef.position.Set(0,0);

        b2Body* body = world->CreateBody(&robotBodyDef);
        worldDat.dynamicBodies.push_back(body);
    }

    obj->setStaticBodies(worldDat.staticBodies);
    QVector<b2JointDef*> joints = obj->setDynamicBodies(worldDat.dynamicBodies);
    for(b2JointDef* j : joints)
    {
        b2Joint* joint = world->CreateJoint(j);
        worldDat.joints.push_back(joint);
    }

    obj->worldTicked(world, 0);
}

void BasicPhysics::removeWorldObject(object_id oId)
{
    if(objects.contains(oId))
    {
        objectWorldData& dat = objects[oId];

        dat.obj->clearDynamicBodies();
        dat.obj->clearStaticBodies();

        for(b2Joint* j : dat.joints)
            world->DestroyJoint(j);

        for(b2Body* s : dat.staticBodies)
            world->DestroyBody(s);

        for(b2Body* d : dat.dynamicBodies)
            world->DestroyBody(d);

       objects.remove(oId);
    }
}

void BasicPhysics::step()
{
    world->Step(stepTime, 8, 3); //suggested values for velocity and position iterations

    emit worldTick(world, stepTime);
}
