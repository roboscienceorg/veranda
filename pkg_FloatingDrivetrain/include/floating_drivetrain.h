#ifndef FLOATER_DRIVETRAIN_H
#define FLOATER_DRIVETRAIN_H

#include <sdsmt_simulator/drivetrain_if.h>

#include <QVector>
#include <QString>

class Floating_Drivetrain : public DriveTrain_If
{
    Q_OBJECT

public:
    Floating_Drivetrain(QObject* parent=nullptr);

    //Gets descriptions for the channels this
    //mediator uses; should map 1-1 to the return of getChannelList
    virtual QVector<QString> getChannelDescriptions();

    //Gets the current names of ROS topics to use
    virtual QVector<QString> getChannelList();

    //Sets the names of ROS topics to use
    virtual void setChannelList(QVector<QString>& channels);

public slots:
    //Tells the drivetrain the speed it's actually going, in local coordinates
    //Should be used for feedback to control code
    virtual void actualVelocity(double xDot, double yDot, double thetaDot);

    //Connects to all ROS topics
    virtual void connectToROS();

    //Disconnects all ROS topics
    virtual void disconnectFromROS();
};

#endif // FLOATER_DRIVETRAIN_H
