#include "basic_physics.h"

BasicPhysics::BasicPhysics(QObject *parent) : Simulator_Physics_If(parent)
{
    //No gravity for top down simulation
    b2Vec2 gravity(0.0f, 0.0f);
    //Create world
    world = new b2World(gravity);

    tickRate = 1;
    stepTime = 1;

    connect(&tick, &QTimer::timeout, this, &BasicPhysics::step);
}

void BasicPhysics::start()
{
    tick.setInterval(1000.0f / rate_hz);
    tick.setTimerType(Qt::PreciseTimer);

    tick.start();
}

void BasicPhysics::stop()
{
    tick.stop();
}

void BasicPhysics::clear()
{
    delete world;
    world = new b2World(gravity);
}

void BasicPhysics::setTick(double rate_hz, double duration_s)
{
    tickRate = rate_hz;
    stepTime = duration_s;
}

void BasicPhysics::newStaticShapes(QVector<b2Shape *> shapes)
{
    clear();
    for(int i = 0; i < shapes.size(); i++)
    {
        b2BodyDef staticBodyDef;
        b2Vec2 position;
        if(shapes[i]->GetType() == b2Shape.e_circle)
            position = shapes[i]->m_p;
        else
            position = shapes[i]->m_centroid;
        staticBodyDef.position.Set(position.x, position.y);
        b2Body* staticBody = world.CreateBody(&staticBodyDef);
        staticBody->CreateFixture(&shapes[i], 0.0f);
    }
}

void BasicPhysics::addRobot(Robot_Physics *robot)
{
    connect(robot, &Robot_Physics::targetVelocityChanged, this, &BasicPhysics::changeTargetVelocity);

    b2BodyDef robotBodyDef;
    robotBodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(robot->xPos, robot->yPos); //robot doesn't have position, but it needs it
    b2Body* robotBody = world.CreateBody(&robotBodyDef);
    b2FixtureDef robotFixtureDef;
    robotFixtureDef.shape = robot->getBodyShape();
    robotFixtureDef.density = 1.0f;
    robotFixtureDef.friction = 0.0f; //CONSIDER CHANGING
    robotBody->CreateFixture(&robotFixtureDef);
    robotWorldData r;
    r.rID = robot->getRobotId();
    r.robotBody = robotBody;
    robots.push_back(r);
}

void BasicPhysics::removeRobot(robot_id rId)
{
    for(int i = 0; i < robots.size(); i++)
        if(robots[i].rID == rId)
        {
            world->DestroyBody(robots[i].robotBody);
            robots.erase[i];
        }
}

void BasicPhysics::changeTargetVelocity(robot_id rId, double xDot, double yDot, double thetaDot)
{
    for(int i = 0; i < robots.size(); i++)
        if(robots[i].rID == rId)
        {
            robots[i].xDot = xDot;
            robots[i].yDot = yDot;
            robots[i].thetaDot = thetaDot;
        }
}

void BasicPhysics::step()
{
    for(int i = 0; i < robots.size(); i++)
    {
        b2Vec2 velocity(robots[i].xDot, robots[i].yDot);
        robots[i].robotBody->SetLinearVelocity(velocity);
    }
    world.step(stepTime, 8, 3); //suggested values for velocity and position iterations
}
