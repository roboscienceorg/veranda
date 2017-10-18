#ifndef FLOATER_DRIVETRAIN_H
#define FLOATER_DRIVETRAIN_H

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include <sdsmt_simulator/drivetrain_if.h>

#include <QVector>
#include <QString>
#include <QObject>

class Floating_Drivetrain : public DriveTrain_If
{
    Q_OBJECT

    QString _velocityChannel;
    bool _connected = false;

    ros::NodeHandle _rosNode;
    ros::Subscriber _listenChannel;

public:
    Floating_Drivetrain(QObject* parent=nullptr);

    virtual QVector<b2Shape*> getModel();

    //Gets descriptions for the channels this
    //mediator uses; should map 1-1 to the return of getChannelList
    virtual QVector<QString> getChannelDescriptions();

    //Gets the current names of ROS topics to use
    virtual QVector<QString> getChannelList();

    //Sets the names of ROS topics to use
    virtual void setChannelList(QVector<QString>& channels);

signals:
    void _incomingMessageSi(std_msgs::Float64MultiArray);

private slots:
    void _incomingMessageSl(std_msgs::Float64MultiArray data);

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
