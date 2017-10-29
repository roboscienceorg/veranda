#include "robot.h"

#include <QDebug>

Robot::Robot(QVector<b2Shape *> body, DriveTrain_If* dt, QVector<Sensor_If*> sensors, QObject* parent) :
    PropertyObject_If(parent), _drivetrain(dt)
{
    connect(_drivetrain, &DriveTrain_If::targetVelocity, this, &Robot::targetVelocity);

    _model = body;

    QMap<QString, PropertyView> props = dt->getAllProperties();
    for(auto iter = props.begin(); iter != props.end(); iter++)
        _properties.insert(dt->propertyGroupName() + "/" + iter.key(), iter.value());

    for(Sensor_If* s : sensors)
    {
        props = s->getAllProperties();
        for(auto iter = props.begin(); iter != props.end(); iter++)
            _properties.insert(s->propertyGroupName() + "/" + iter.key(), iter.value());
    }
}

void Robot::setPhysicsBody(b2Body* body)
{
    if(body != nullptr)
    {
        for(b2Shape* s : _model)
        {
            b2FixtureDef bodyDef;
            bodyDef.shape = s;
            bodyDef.density = 1.0;
            bodyDef.friction = 0.0;

            body->CreateFixture(&bodyDef);
        }
    }
    _bodyPhysics = body;
}

void Robot::connectToROS()
{
    _drivetrain->connectToROS();
}

void Robot::disconnectFromROS()
{
    _drivetrain->disconnectFromROS();
}

void Robot::targetVelocity(double x, double y, double theta)
{
    if(_bodyPhysics)
    {
        _bodyPhysics->SetLinearVelocity(b2Vec2(x, y));
        _bodyPhysics->SetAngularVelocity(theta);
    }
}

void Robot::worldTicked(const double t, const b2World* world)
{
    if(_bodyPhysics)
    {
        _drivetrain->worldTicked(t, world, _bodyPhysics);
        for(Sensor_If* s : _sensors) s->worldTicked(t, world, _bodyPhysics);

        _x = _bodyPhysics->GetWorldCenter().x;
        _y = _bodyPhysics->GetWorldCenter().y;
        _theta = _bodyPhysics->GetAngle();

        qDebug() << _x << _y << _theta;

        _newPosition(_x, _y, _theta);
    }
}
