#ifndef BASIC_PHYSICS_H
#define BASIC_PHYSICS_H

#include "interfaces/simulator_physics_if.h"
#include <QTimer>

#include <QObject>

struct robotWorldData{
    b2Body *robotBody;
    Robot_Physics *robot;
    double xDot;
    double yDot;
    double thetaDot;
};

class BasicPhysics : public Simulator_Physics_If
{
    Q_OBJECT

    b2World *world;

    QVector<robotWorldData> robots;

    QTimer tick;

    double tickRate;
    double stepTime;


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

    void changeTargetVelocity(robot_id rId, double xDot, double yDot, double thetaDot);

    void step();
};

#endif // BASIC_PHYSICS_H
