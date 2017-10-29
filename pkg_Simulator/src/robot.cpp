#include "robot.h"

#include <QDebug>

Robot::Robot(b2Shape* body, DriveTrain_If* dt, QVector<Sensor_If*> sensors, QObject* parent) : PropertyObject_If(parent), _body(body), _drivetrain(dt)
{
    connect(_drivetrain, &DriveTrain_If::targetVelocity, this, &Robot::targetVelocity);

    QMap<QString, PropertyView> props = dt->getAllProperties();
    for(auto iter = props.begin(); iter != props.end(); iter++)
        _properties.insert(dt->propertyGroupName() + "/" + iter.key(), iter.value());

    for(Sensor_If* s : sensors)
    {
        props = s->getAllProperties();
        for(auto iter = props.begin(); iter != props.end(); iter++)
            _properties.insert(s->propertyGroupName() + "/" + iter.key(), iter.value());
    }

    //qDebug() << "Robot properties";
    //for(auto& p : _properties)
        //qDebug() << &p;
}

const b2Shape* Robot::getRobotBody()
{
    return _body;
}

const QVector<b2Shape*>& Robot::getRobotModel()
{
    return _model;
}

void Robot::connectToROS()
{
    _drivetrain->connectToROS();
}

void Robot::disconnectFromROS()
{
    _drivetrain->disconnectFromROS();
}

void Robot::actualVelocity(double xDot, double yDot, double thetaDot)
{

}

void Robot::actualPosition(double x, double y, double theta)
{
    _x = x;
    _y = y;
    _theta = theta;
    _newPosition(_x, _y, _theta);
}

void Robot::worldTicked()
{

}
