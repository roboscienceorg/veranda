#ifndef ROBOTCOMPONENT_IF_H
#define ROBOTCOMPONENT_IF_H

#include <QObject>

#include <Box2D/Box2D.h>

class RobotComponent_If : public QObject
{
    Q_OBJECT

public:
    RobotComponent_If(QObject* parent=nullptr) : QObject(parent){}

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

public slots:
    //Connects to all ROS topics
    virtual void connectToROS() = 0;

    //Disconnects all ROS topics
    virtual void disconnectFromROS() = 0;
};

#endif // ROBOTCOMPONENT_IF_H
