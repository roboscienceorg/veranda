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
    Robot_Interface(Robot_Interface* interface) : _observed(interface->_observed), _id(interface->_id){}

    robot_id getRobotId() { return _id; }
};

class Robot_Physics : public QObject, public Robot_Interface
{
    Q_OBJECT

    const b2Shape* _body;

    void _init()
    {
        qRegisterMetaType<robot_id>("robot_id");

        connect(this, &Robot_Physics::setActualPosition, _observed, &Robot::actualPosition);
        connect(this, &Robot_Physics::setActualVelocity, _observed, &Robot::actualVelocity);
        connect(this, &Robot_Physics::notifyWorldTicked, _observed, &Robot::worldTicked);

        connect(_observed, &Robot::targetVelocity, [this](double x, double y, double z){targetVelocityChanged(_id, x, y, z);});

        _body = _observed->getRobotBody();
    }

public:
    Robot_Physics(Robot* observed, robot_id id) : QObject(observed), Robot_Interface(observed, id)
    {
        _init();
    }

    Robot_Physics(Robot_Physics* interface) : QObject(interface->_observed), Robot_Interface(interface)
    {
        _init();
    }

    const b2Shape* getBodyShape(){ return _body; }

signals:
   /****************************************************************
    * Connect For Data From Robot
    ****************************************************************/

    //Signals velocity that this robot wants to go, in global coordinates
    void targetVelocityChanged(robot_id, double xDot, double yDot, double thetaDot);

    /****************************************************************
     * Call To Update Robot
     ****************************************************************/

    //Tells the robot the speed it's actually going, in global coordinates
    //Should be used for feedback to control code
    void setActualVelocity(double xDot, double yDot, double thetaDot);

    //Tells the robot it's world-space position
    void setActualPosition(double x, double y, double theta);

    //Tells the robot that the world has updated
    void notifyWorldTicked();
};

class Robot_Properties : public QObject, public Robot_Interface
{
    Q_OBJECT

    void _init()
    {

    }

public:
    Robot_Properties(Robot* observed, robot_id id) : QObject(observed), Robot_Interface(observed, id)
    {
        _init();
    }

    Robot_Properties(Robot_Properties* interface) : QObject(interface->_observed), Robot_Interface(interface)
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

signals:
    /****************************************************************
     * From Robot
     ****************************************************************/

    void robotPropertiesChanged(QVariantMap properties);

    /****************************************************************
     * To Robot
     ****************************************************************/

    void setRobotProperty(QString property, QVariant value);

    void disconnectRobotFromROS();
    void connectRobotToROS();
};

#endif // ROBOT_INTERFACES_H
