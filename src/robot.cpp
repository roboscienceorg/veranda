#include "robot.h"

Robot::Robot(b2Shape* body, DriveTrain_If* dt, QVector<Sensor_If*> sensors, QObject* parent) : QObject(parent)
{

}

const b2Shape* Robot::getBodyShape()
{

}

QVector<QString> Robot::getChannelDescriptions()
{
    return QVector<QString>();
}

QVector<QString> Robot::getChannelList()
{
    return QVector<QString>();
}

void Robot::setChannelList(QVector<QString>& channels)
{

}

void Robot::connectToROS()
{

}

void Robot::disconnectFromROS()
{

}

void Robot::actualVelocity(double xDot, double yDot, double thetaDot)
{

}

void Robot::actualPosition(double x, double y, double theta)
{

}

void Robot::worldTicked()
{

}
