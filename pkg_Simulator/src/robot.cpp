#include "robot.h"

#include <QDebug>

Robot::Robot(QVector<b2Shape *> body, DriveTrain_If* dt, double x0, double y0, double theta0, QVector<Sensor_If*> sensors, QObject* parent) :
    WorldObject_If(parent), _x0(x0), _y0(y0), _theta0(theta0*DEG2RAD), _drivetrain(dt)
{
    _model = new Model({}, body, this);
    _allModels.push_back(_model);

    connect(_drivetrain, &DriveTrain_If::targetVelocity, this, &Robot::targetVelocity);
    _bodiesRequired += dt->dynamicBodiesRequired();
    _drivetrain->setParent(this);

    QMap<QString, PropertyView> props = dt->getAllProperties();
    for(auto iter = props.begin(); iter != props.end(); iter++)
    {
        _properties.insert(dt->propertyGroupName() + "/" + iter.key(), iter.value());
    }
    _model->addChildren(_drivetrain->getModels());

    for(Sensor_If* s : sensors)
    {
        _bodiesRequired += s->dynamicBodiesRequired();
        s->setParent(this);

        props = s->getAllProperties();
        for(auto iter = props.begin(); iter != props.end(); iter++)
            _properties.insert(s->propertyGroupName() + "/" + iter.key(), iter.value());
        _model->addChildren(s->getModels());
    }
}

Robot::~Robot()
{

}

WorldObject_If* Robot::clone(QObject *newParent)
{
    DriveTrain_If* newDt = static_cast<DriveTrain_If*>(_drivetrain->clone());

    QVector<Sensor_If*> newSensors;
    for(Sensor_If* s : _sensors)
        newSensors.push_back(static_cast<Sensor_If*>(s->clone()));

    QVector<b2Shape*> newBody;
    for(b2Shape* s : _model->shapes())
        newBody.push_back(cloneShape(s));

    return new Robot(newBody, newDt, _x0, _y0, _theta0*RAD2DEG, newSensors, newParent);
}

void Robot::clearDynamicBodies()
{
    _drivetrain->clearDynamicBodies();
    for(Sensor_If* s : _sensors)
        s->clearDynamicBodies();
    _mainBody = nullptr;
}

QVector<b2JointDef *> Robot::setDynamicBodies(QVector<b2Body*>& bodies)
{
    QVector<b2JointDef*> joints;

    QVector<b2Body*> tmp;
    int i=0;
    for(int j=0; j< _drivetrain->dynamicBodiesRequired(); j++, i++)
        tmp.push_back(bodies.at(j));
    joints += _drivetrain->setDynamicBodies(tmp);

    for(Sensor_If* s : _sensors)
    {
        tmp.clear();
        for(int j=0; j<s->dynamicBodiesRequired(); j++, i++)
            tmp.push_back(bodies.at(j));
        joints += s->setDynamicBodies(tmp);
    }

    _mainBody = bodies.at(i);
    for(b2Shape* s : _model->shapes())
    {
        b2FixtureDef bodyDef;
        bodyDef.shape = s;
        bodyDef.density = 1.0;
        bodyDef.friction = 0.0;

        _mainBody->CreateFixture(&bodyDef);
    }
    _mainBody->SetLinearVelocity(b2Vec2(0,0));
    _mainBody->SetAngularVelocity(0);
    _mainBody->SetTransform(b2Vec2(_x0, _y0), _theta0);
    qDebug() << "Starting robot at " << _x0 << _y0 << _theta0;

    return joints;
}

void Robot::connectChannels()
{
    _drivetrain->connectChannels();
    for(Sensor_If* s : _sensors)
        s->connectChannels();
}

void Robot::disconnectChannels()
{
    _drivetrain->disconnectChannels();
    for(Sensor_If* s : _sensors)
        s->disconnectChannels();
}

void Robot::targetVelocity(double x, double y, double theta)
{
    if(_mainBody)
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

        _mainBody->SetLinearVelocity(b2Vec2(x, y));
        _mainBody->SetAngularVelocity(theta);
    }
}

void Robot::worldTicked(const b2World* world, const double& t)
{
    if(_mainBody)
    {
        _drivetrain->worldTicked(world, t);
        for(Sensor_If* s : _sensors) s->worldTicked(world, t);

        _x = _mainBody->GetWorldCenter().x;
        _y = _mainBody->GetWorldCenter().y;
        _theta = _mainBody->GetAngle();

        while(_theta < 0) _theta += 2*PI;
        while(_theta > 2*PI) _theta -= 2*PI;

        //qDebug() << _x << _y << _theta;

        _model->setTransform(_x, _y, _theta*RAD2DEG);
    }
}
