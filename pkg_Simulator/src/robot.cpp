#include "robot.h"

#include <QDebug>

Robot::Robot(b2Shape* body, DriveTrain_If* dt, QVector<Sensor_If*> sensors, QObject* parent) : QObject(parent), _body(body), _drivetrain(dt)
{
    connect(_drivetrain, &DriveTrain_If::targetVelocity, this, &Robot::targetVelocity);
}

const b2Shape* Robot::getRobotBody()
{
    return _body;
}

const QVector<b2Shape*>& Robot::getRobotModel()
{
    return _model;
}

QVector<QString> Robot::getChannelDescriptions()
{
    return QVector<QString>();
}

QVector<QString> Robot::getChannelList()
{
    return QVector<QString>();
}

void Robot::setChannelList(const QVector<QString>& channels)
{
    _drivetrain->setChannelList({channels[0]});
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
