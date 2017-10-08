#ifndef BASIC_PHYSICS_H
#define BASIC_PHYSICS_H

#include "interfaces/simulator_physics_if.h"

#include <QObject>

class BasicPhysics : public Simulator_Physics_If
{
    Q_OBJECT

public:
    BasicPhysics(QObject* parent = nullptr);

public slots:
    virtual void start() override;
    virtual void stop() override;
    virtual void clear() override;

    //Set tick rate and duration
    virtual void setTick(double rate_hz, double duration_s) override;

    //Sets the static shapes in the simulation
    virtual void newStaticShapes(QVector<b2Shape*> shapes) override;

    //Adds a new robot to the simulation
    //The simulator should add the robot body as a dynamic shape
    //And connect signals to the robot slots
    //Do not delete the robot interface when the robot is removed; it will be handled elsewhere
    virtual void addRobot(Robot_Physics* robot) override;

    //Removes a robot from simulation
    virtual void removeRobot(robot_id rId) override;
};

#endif // BASIC_PHYSICS_H
