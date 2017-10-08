#ifndef SIMULATOR_PHYSICS_H
#define SIMULATOR_PHYSICS_H

#include <QObject>
#include <QVector>

#include <Box2D/Box2D.h>

#include "robot_interfaces.h"

class Simulator_Physics_If : public QObject
{
    Q_OBJECT

public:
    Simulator_Physics_If(QObject* parent = nullptr) : QObject(parent){}

public slots:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void clear() = 0;

    //Set tick rate and duration
    virtual void setTick(double rate_hz, double duration_s) = 0;

    //Sets the static shapes in the simulation
    virtual void newStaticShapes(QVector<b2Shape*> shapes) = 0;

    //Adds a new robot to the simulation
    //The simulator should add the robot body as a dynamic shape
    //And connect signals to the robot slots
    //Do not delete the robot interface when the robot is removed; it will be handled elsewhere
    virtual void addRobot(Robot_Physics* robot) = 0;

    //Removes a robot from simulation
    virtual void removeRobot(robot_id rId) = 0;
};

#endif // SIMULATOR_PHYSICS_H
