#ifndef ROBOT_INTERFACES_H
#define ROBOT_INTERFACES_H

#include "robot.h"
#include "screen_model_if.h"

#include <QVariantMap>

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
class Robot_Interface
{

protected:
    Robot* _observed;
    robot_id _id;

public:
    Robot_Interface(Robot* observed, robot_id id) : _observed(observed), _id(id){}

    robot_id getRobotId() { return _id; }
};

class Robot_Physics : public QObject, public Robot_Interface
{
    Q_OBJECT

    void _init()
    {
        connect(this, &Robot_Physics::notifyWorldTicked, _observed, &Robot::worldTicked);
    }

public:
    Robot_Physics(Robot* observed, robot_id id) : QObject(observed), Robot_Interface(observed, id)
    {
        _init();
    }

    //Sets b2Body for robot; robot populates with fixtures
    void setPhysicsBody(b2Body* body)
    {
        _observed->setPhysicsBody(body);
    }

signals:
    //Tells the robot that the world has updated
    void notifyWorldTicked(const double t, const b2World*);
};

class Robot_Properties : public PropertyObject_If, public Robot_Interface
{
    Q_OBJECT

    void _init()
    {
        connect(this, &Robot_Properties::disconnectRobotFromROS, _observed, &Robot::disconnectFromROS);
        connect(this, &Robot_Properties::connectRobotToROS, _observed, &Robot::connectToROS);
    }

public:
    Robot_Properties(Robot* observed, robot_id id, QObject* parent=nullptr) : PropertyObject_If(observed), Robot_Interface(observed, id)
    {
        _init();
    }

    ScreenModel_If* createRobotBaseModel()
    {
        return new RobotBaseScreenModel(_observed);
    }

    ScreenModel_If* createRobotSensorsModel()
    {
        return new RobotSensorsScreenModel(_observed);
    }

    QString propertyGroupName(){ return _observed->propertyGroupName(); }
    QMap<QString, PropertyView>& getAllProperties(){ return _observed->getAllProperties(); }

signals:
    void disconnectRobotFromROS();
    void connectRobotToROS();
};

#endif // ROBOT_INTERFACES_H
