#include "diffDrive_drivetrain.h"
#include "std_msgs/Float64.h"
#include <QDebug>

DiffDrive_Drivetrain::DiffDrive_Drivetrain(QObject *parent) : DriveTrain_If(parent)
{
    connect(this, &DiffDrive_Drivetrain::_incomingMessageSi, this, &DiffDrive_Drivetrain::_incomingMessageSl);
    connect(&velocity_channel, &Property::valueSet, this, &DiffDrive_Drivetrain::_channelChanged);

    qRegisterMetaType<std_msgs::Float64MultiArray>("std_msgs::Float64MultiArray");
}

void DiffDrive_Drivetrain::_channelChanged(QVariant)
{
    if(_connected)
    {
        disconnectChannels();
        connectChannels();
    }
}

void DiffDrive_Drivetrain::actualVelocity(double xDot, double yDot, double thetaDot)
{
    velocity_x.set(xDot);
    velocity_y.set(yDot);
    velocity_theta.set(thetaDot);
}

void DiffDrive_Drivetrain::connectChannels()
{
    if(_connected)
        disconnectChannels();

    _listenChannel = _rosNode.subscribe(velocity_channel.get().toString().toStdString(), 10, &DiffDrive_Drivetrain::_incomingMessageSi, this);
    _connected = true;
}

void DiffDrive_Drivetrain::disconnectChannels()
{
    _listenChannel.shutdown();
    _connected = false;
}

void DiffDrive_Drivetrain::_incomingMessageSl(std_msgs::Float64MultiArray data)
{
    if(sizeof(data.data) >= 2 * sizeof(std_msgs::Float64))
    {
        double x, y, theta;
        FK(data.data[0], data.data[1], x, y, theta);

        velocity_x.set(x);
        velocity_y.set(y);
        velocity_theta.set(theta);

        targetVelocity(x, y, theta);
    }
}

void DiffDrive_Drivetrain::FK(const double& phi1, const double& phi2, double& xDot, double& yDot, double& thetaDot)
{
    double r = wheel_radius.get().toDouble();
    double l = axle_length.get().toDouble();

    yDot = 0;
    xDot = r/2.0 * (phi1 + phi2);
    thetaDot = r/(2.0*l) * (phi1 - phi2);
}
