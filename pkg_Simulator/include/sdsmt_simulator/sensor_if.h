#ifndef SENSOR_H
#define SENSOR_H

#include <Box2D/Box2D.h>

#include <QVector>
#include <QObject>

#include "robotcomponent_if.h"

class Sensor_If : public RobotComponent_If
{
    Q_OBJECT

public:
    Sensor_If(QObject* parent = nullptr) : RobotComponent_If(parent){}

    //Gets descriptions for the channels this
    //mediator uses; should map 1-1 to the return of getChannelList
    virtual QVector<QString> getChannelDescriptions() = 0;

    //Gets the current names of ROS topics to use
    virtual QVector<QString> getChannelList() = 0;

    //Sets the names of ROS topics to use
    virtual void setChannelList(QVector<QString>& channels) = 0;

public slots:
    virtual void worldTicked() = 0;

    //Connects to all ROS topics
    virtual void connectToROS() = 0;

    //Disconnects all ROS topics
    virtual void disconnectFromROS() = 0;

signals:
    void visualChanged(QVector<b2Shape*> visualObjects);
};
#endif // SENSOR_H
