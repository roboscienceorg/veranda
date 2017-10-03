#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>

#include "interfaces/sensor_if.h"
#include "interfaces/drivetrain_if.h"

#include <Box2D/Box2D.h>

typedef uint64_t robot_id;

class Robot : public QObject
{
    Q_OBJECT

public:
    Robot(b2Shape* body, DriveTrain_If* dt, QVector<Sensor_If*> sensors, QObject* parent = nullptr);

    const b2Shape* getBodyShape();

    //Gets descriptions for the channels this
    //mediator uses; should map 1-1 to the return of getChannelList
    virtual QVector<QString> getChannelDescriptions();

    //Gets the current names of ROS topics to use
    virtual QVector<QString> getChannelList();

    //Sets the names of ROS topics to use
    virtual void setChannelList(QVector<QString>& channels);

public slots:
    //Tells the robot to connect all its ROS topics
    void connectToROS();

    //Tells the robot to disconnect all its ROS topics
    void disconnectFromROS();

    //Tells the robot the speed it's actually going, in global coordinates
    //Should be used for feedback to control code
    void actualVelocity(double xDot, double yDot, double thetaDot);

    //Tells the robot it's world-space position
    void actualPosition(double x, double y, double theta);

    //Tells the robot that the world has updated
    void worldTicked();

signals:
    //Signals velocity that this robot wants to go, in global coordinates
    void targetVelocity(double xDot, double yDot, double thetaDot);
};

#endif // ROBOT_H
