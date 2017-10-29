#ifndef FLOATER_DRIVETRAIN_H
#define FLOATER_DRIVETRAIN_H

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include <sdsmt_simulator/drivetrain_if.h>
#include <sdsmt_simulator/properties_if.h>

#include <QVector>
#include <QString>
#include <QObject>
#include <QMap>

class Floating_Drivetrain : public DriveTrain_If
{
    Q_OBJECT

    bool _connected = false;

    ros::NodeHandle _rosNode;
    ros::Subscriber _listenChannel;

    Property velocity_channel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                      "Input channel for velocity of the craft in world coords (x, y, theta)"), "");

    Property velocity_x = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE,
                                     "Velocity of the craft in the x direction"), QVariant(0));

    Property velocity_y = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE,
                                     "Velocity of the craft in the y direction"), QVariant(0));

    Property velocity_theta = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE,
                                         "Rotational velocity of the craft"), QVariant(0));

    QMap<QString, PropertyView> _properties{
        {"channels/input_velocities", PropertyView(&velocity_channel)},
        {"velocity/x", PropertyView(&velocity_x)},
        {"velocity/y", PropertyView(&velocity_y)},
        {"velocity/theta", PropertyView(&velocity_theta)}
    };

public:
    Floating_Drivetrain(QObject* parent=nullptr);

    virtual QVector<b2Shape*> getModel();

    virtual bool usesWorldCoords(){return true;}

    virtual QMap<QString, PropertyView>& getAllProperties(){
        return _properties;
    }

    virtual QString propertyGroupName(){ return "Float Drive"; }

signals:
    void _incomingMessageSi(std_msgs::Float64MultiArray);

private slots:
    void _incomingMessageSl(std_msgs::Float64MultiArray data);
    void _channelChanged(QVariant);

public slots:
    //Tells the drivetrain the speed it's actually going, in local coordinates
    //Should be used for feedback to control code
    virtual void actualVelocity(double xDot, double yDot, double thetaDot);

    //Connects to all ROS topics
    virtual void connectToROS();

    //Disconnects all ROS topics
    virtual void disconnectFromROS();
    virtual void worldTicked(const double t, const b2World*, const b2Body*){}
};

#endif // FLOATER_DRIVETRAIN_H
