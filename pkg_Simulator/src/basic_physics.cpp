#include "basic_physics.h"
#include <QDebug>
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

void BasicPhysics::newStaticShapes(QVector<b2Shape *> shapes)
{
    clear();
    for(int i = 0; i < shapes.size(); i++)
    {
        b2BodyDef staticBodyDef;
        b2Vec2 position;
        if(shapes[i]->GetType() == 0)
            position = static_cast<b2CircleShape*>(shapes[i])->m_p;
        else
            position = static_cast<b2PolygonShape*>(shapes[i])->m_centroid;
        staticBodyDef.position.Set(position.x, position.y);
        b2Body* staticBody = world->CreateBody(&staticBodyDef);
        staticBody->CreateFixture(shapes[i], 0.0f);
    }
}

void BasicPhysics::addRobot(Robot_Physics *robot)
{
    b2BodyDef robotBodyDef;
    robotBodyDef.type = b2_dynamicBody;
    robotBodyDef.position.Set(10.0f, 10.0f);
    b2Body* robotBody = world->CreateBody(&robotBodyDef);

    robot->setPhysicsBody(robotBody);

    robotWorldData r;
    r.robot = robot;
    r.robotBody = robotBody;
    robots.push_back(r);

    robot->notifyWorldTicked(0, world);
}

void BasicPhysics::removeRobot(robot_id rId)
{
    for(int i = 0; i < robots.size(); i++)
        if(robots[i].robot->getRobotId() == rId)
        {
            robots[i].robot->setPhysicsBody(nullptr);
            world->DestroyBody(robots[i].robotBody);
            robots.erase(robots.begin() + i);
        }
}

void BasicPhysics::step()
{
    world->Step(stepTime, 8, 3); //suggested values for velocity and position iterations
    for(int i = 0; i < robots.size(); i++)
    {
        robots[i].robot->notifyWorldTicked(stepTime, world);
    }
}
