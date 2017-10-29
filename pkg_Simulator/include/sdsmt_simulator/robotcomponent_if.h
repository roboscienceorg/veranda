#ifndef ROBOTCOMPONENT_IF_H
#define ROBOTCOMPONENT_IF_H

#include "properties_if.h"

#include <QObject>

#include <Box2D/Box2D.h>

class RobotComponent_If : public PropertyObject_If
{
    Q_OBJECT

protected:
    const b2World* _world = nullptr;

public:
    RobotComponent_If(QObject* parent=nullptr) : PropertyObject_If(parent){}

    //Returns a set of shapes that are the
    //base model for this component
    virtual QVector<b2Shape*> getModel() = 0;

public slots:
    virtual void worldTicked(const double t, const b2World*, const b2Body*) = 0;

    //Connects to all ROS topics
    virtual void connectToROS() = 0;

    //Disconnects all ROS topics
    virtual void disconnectFromROS() = 0;
};


#endif // ROBOTCOMPONENT_IF_H
