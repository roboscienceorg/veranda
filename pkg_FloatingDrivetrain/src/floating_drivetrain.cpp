#include "floating_drivetrain.h"
#include "std_msgs/Float64.h"
#include <QDebug>

Floating_Drivetrain::Floating_Drivetrain(QObject *parent) : DriveTrain_If(parent)
{
    connect(this, &Floating_Drivetrain::_incomingMessageSi, this, &Floating_Drivetrain::_incomingMessageSl);
    connect(&velocity_channel, &Property::valueSet, this, &Floating_Drivetrain::_channelChanged);

    qRegisterMetaType<std_msgs::Float64MultiArray>("std_msgs::Float64MultiArray");

    //qDebug() << "Floating drivetrain properties";
    //for(auto& p : _properties)
        //qDebug() << &p;
}

void Floating_Drivetrain::_channelChanged(QVariant channel)
{
    //qDebug() << "Floating drivetrain channel changed to " << channel;
    if(_connected)
    {
        disconnectChannels();
        connectChannels();
    }
}

void Floating_Drivetrain::actualVelocity(double xDot, double yDot, double thetaDot)
{
    velocity_x.set(xDot);
    velocity_y.set(yDot);
    velocity_theta.set(thetaDot);
}

void Floating_Drivetrain::connectChannels()
{
    if(_connected)
        disconnectChannels();

    _listenChannel = _rosNode.subscribe(velocity_channel.get().toString().toStdString(), 10, &Floating_Drivetrain::_incomingMessageSi, this);
    _connected = true;
}

void Floating_Drivetrain::disconnectChannels()
{
    _listenChannel.shutdown();
    _connected = false;
}


void Floating_Drivetrain::_incomingMessageSl(std_msgs::Float64MultiArray data)
{
    if(sizeof(data.data) >= 3 * sizeof(std_msgs::Float64))
    {
        velocity_x.set(data.data[0]);
        velocity_y.set(data.data[1]);
        velocity_theta.set(data.data[2]);

        targetVelocity(data.data[0], data.data[1], data.data[2]);
    }
}
