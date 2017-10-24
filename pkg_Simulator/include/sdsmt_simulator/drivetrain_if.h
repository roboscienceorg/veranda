#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <QObject>

#include "robotcomponent_if.h"

class DriveTrain_If : public RobotComponent_If
{
    Q_OBJECT

public:
    DriveTrain_If(QObject* parent = nullptr) : RobotComponent_If(parent){}

    //Returns true if the plugin treats input and output velocities
    //as being in world coordinates. Otherwise, they are in robot coordinates
    virtual bool usesWorldCoords() = 0;

signals:
    //Signals velocity that this drivetrain wants to go, in local coordinates
    void targetVelocity(double xDot, double yDot, double thetaDot);

public slots:
    //Tells the drivetrain the speed it's actually going, in local coordinates
    //Should be used for feedback to control code
    virtual void actualVelocity(double xDot, double yDot, double thetaDot) = 0;
};

#endif // DRIVETRAIN_H
