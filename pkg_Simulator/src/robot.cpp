#include "robot.h"

#include <QDebug>

Robot::Robot(QVector<b2Shape *> body, DriveTrain_If* dt, double x0, double y0, double theta0, QVector<Sensor_If*> sensors, QObject* parent) :
    PropertyObject_If(parent), _x0(x0), _y0(y0), _theta0(theta0*DEG2RAD), _drivetrain(dt)
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
        body->SetLinearVelocity(b2Vec2(0,0));
        body->SetAngularVelocity(0);
        body->SetTransform(b2Vec2(_x0, _y0), _theta0);
        qDebug() << "Starting robot at " << _x0 << _y0 << _theta0;
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
        //Translate to world coordinate velocities
        if(!_drivetrain->usesWorldCoords())
        {
            double sint = sin(_theta);
            double cost = cos(_theta);

            //qDebug() << "Robot velocity to world velocity" << _theta << sint << cost;
            //qDebug() << x << y;

            double x_ = cost*x + -sint*y;
            y = sint*x + cost*y;
            x = x_;

            //qDebug() << x << y;
        }
        if(_drivetrain->usesDegrees())
        {
            theta *= DEG2RAD;
        }

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

        while(_theta < 0) _theta += 2*PI;
        while(_theta > 2*PI) _theta -= 2*PI;

        //qDebug() << _x << _y << _theta;

        _newPosition(_x, _y, _theta*RAD2DEG);
    }
}
