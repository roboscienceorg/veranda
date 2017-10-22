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
    connect(robot, &Robot_Physics::targetVelocityChanged, this, &BasicPhysics::changeTargetVelocity);

    b2BodyDef robotBodyDef;
    robotBodyDef.type = b2_dynamicBody;
    robotBodyDef.position.Set(10.0f, 10.0f);
    b2Body* robotBody = world->CreateBody(&robotBodyDef);
    b2FixtureDef robotFixtureDef;
    robotFixtureDef.shape = robot->getBodyShape();
    robotFixtureDef.density = 1.0f;
    robotFixtureDef.friction = 0.0f; //CONSIDER CHANGING
    robotBody->CreateFixture(&robotFixtureDef);
    robotWorldData r;
    r.robot = robot;
    r.robotBody = robotBody;
    robots.push_back(r);
}

void BasicPhysics::removeRobot(robot_id rId)
{
    for(int i = 0; i < robots.size(); i++)
        if(robots[i].robot->getRobotId() == rId)
        {
            world->DestroyBody(robots[i].robotBody);
            robots.erase(robots.begin() + i);
        }
}

void BasicPhysics::changeTargetVelocity(robot_id rId, double xDot, double yDot, double thetaDot)
{
    for(int i = 0; i < robots.size(); i++)
        if(robots[i].robot->getRobotId() == rId)
        {
            b2Vec2 v(xDot, yDot);
            robots[i].robotBody->SetLinearVelocity(v);
        }
}

void BasicPhysics::step()
{
    world->Step(stepTime, 8, 3); //suggested values for velocity and position iterations
    for(int i = 0; i < robots.size(); i++)
    {
        b2Vec2 v = robots[i].robotBody->GetPosition();
        robots[i].robot->setActualPosition(v.x, v.y, 0.0f);
        robots[i].robot->notifyWorldTicked();
    }
}
