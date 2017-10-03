#ifndef ROBOT_INTERFACES_H
#define ROBOT_INTERFACES_H

#include "robot.h"

/*************************************
 * Interface classes for interacting with robots
 *
 * QObject doesn't work well with multiple inheritance
 * so we can't just have Robot inherit multiple interfaces
 * because they would all need their own signals, and therefore
 * would have to be QObjects
 *
 * This solves that issue by creating proxy objects
 * that parts of the program can work with
 *
 * An added bonus is that since all the function calls
 * go through signals, they are guaranteed to be threadsafe
 ****************************************/
class Robot_Physics : public QObject
{
    Q_OBJECT

    b2Shape _body;

public:
    Robot_Physics(Robot* observed, QObject* parent = nullptr) : QObject(parent)
    {
        connect(this, &Robot_Physics::setActualPosition, observed, &Robot::actualPosition);
        connect(this, &Robot_Physics::setActualVelocity, observed, &Robot::actualVelocity);
        connect(this, &Robot_Physics::notifyWorldTicked, observed, &Robot::worldTicked);

        connect(observed, &Robot::targetVelocity, this, &Robot_Physics::targetVelocityChanged);

        _body = observed->getBodyShape();
    }

    const b2Shape getBodyShape(){ return _body; }

signals:
   /****************************************************************
    * From Robot
    ****************************************************************/

    //Signals velocity that this robot wants to go, in global coordinates
    void targetVelocityChanged(double xDot, double yDot, double thetaDot);

    /****************************************************************
     * To Robot
     ****************************************************************/

    //Tells the robot the speed it's actually going, in global coordinates
    //Should be used for feedback to control code
    void setActualVelocity(double xDot, double yDot, double thetaDot);

    //Tells the robot it's world-space position
    void setActualPosition(double x, double y, double theta);

    //Tells the robot that the world has updated
    void notifyWorldTicked();
};

class Robot_Properties : public QObject
{
    Q_OBJECT

public:
    Robot_Properties(Robot* observed, QObject* parent = nullptr) : QObject(parent)
    {

    }

signals:
    /****************************************************************
     * From Robot
     ****************************************************************/

    void robotSelectedChanged(bool isSelected);
    void robotPropertiesChanged(QVariantMap properties);

    /****************************************************************
     * To Robot
     ****************************************************************/

    void setRobotSelected(bool isSelected);
    void setRobotProperty(QString property, QVariant value);

    void disconnectRobotFromROS();
    void connectRobotToROS();
};

class Robot_Visual : public QObject
{
    Q_OBJECT

public:
    Robot_Visual(Robot* observed, QObject* parent = nullptr) : QObject(parent)
    {

    }

    /*[Local-space polygon representations] getRobotModel()*/

signals:
    /****************************************************************
     * From Robot
     ****************************************************************/

    void robotSelectedChanged(bool isSelected);
    void robotPositionChanged(double x, double y, double theta);
    void drawnExtrasChanged(/*QVector<[World-space polygon representations]>*/);

    /****************************************************************
     * To Robot
     ****************************************************************/

    void setRobotSelected(bool isSelected);
};


#endif // ROBOT_INTERFACES_H
