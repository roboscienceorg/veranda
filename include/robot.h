#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>

#include "sensor_if.h"
#include "drivetrain_if.h"

#include <Box2D/Box2D.h>

typedef uint64_t robot_id;

class Robot : public QObject
{
    Q_OBJECT

public:
    Robot(b2Shape body, DriveTrain_If* dt, QVector<Sensor_If*> sensors, QObject* parent = nullptr);

    const b2Shape getBodyShape();

    //Gets descriptions for the channels this
    //mediator uses; should map 1-1 to the return of getChannelList
    virtual QVector<QString> getChannelDescriptions();

    //Gets the current names of ROS topics to use
    virtual QVector<QString> getChannelList();

    //Sets the names of ROS topics to use
    virtual void setChannelList(QVector<QString>& channels);

public slots:
    //Tells the robot to connect all its ROS topics
    void connectToROS();

    //Tells the robot to disconnect all its ROS topics
    void disconnectFromROS();

    //Tells the robot the speed it's actually going, in global coordinates
    //Should be used for feedback to control code
    void actualVelocity(double xDot, double yDot, double thetaDot);

    //Tells the robot it's world-space position
    void actualPosition(double x, double y, double theta);

    //Tells the robot that the world has updated
    void worldTicked();

signals:
    //Signals velocity that this robot wants to go, in global coordinates
    void targetVelocity(double xDot, double yDot, double thetaDot);
};

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

#endif // ROBOT_H
