#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <QObject>

#include "robotcomponent_if.h"

class DriveTrain_If : public RobotComponent_If
{
    Q_OBJECT

public:
    DriveTrain_If(QObject* parent = nullptr) : RobotComponent_If(parent){}

    //Returns a set of shapes that are the
    //base model for this component
    virtual QVector<b2Shape*> getModel() = 0;

    //Gets descriptions for the channels this
    //mediator uses; should map 1-1 to the return of getChannelList
    virtual QVector<QString> getChannelDescriptions() = 0;

    //Gets the current names of ROS topics to use
    virtual QVector<QString> getChannelList() = 0;

    //Sets the names of ROS topics to use
    virtual void setChannelList(QVector<QString>& channels) = 0;

signals:
    //Signals velocity that this drivetrain wants to go, in local coordinates
    void targetVelocity(double xDot, double yDot, double thetaDot);

public slots:
    //Tells the drivetrain the speed it's actually going, in local coordinates
    //Should be used for feedback to control code
    virtual void actualVelocity(double xDot, double yDot, double thetaDot) = 0;

    //Connects to all ROS topics
    virtual void connectToROS() = 0;

    //Disconnects all ROS topics
    virtual void disconnectFromROS() = 0;
};

#endif // DRIVETRAIN_H
