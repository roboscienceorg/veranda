#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>

#include "sdsmt_simulator/sensor_if.h"
#include "sdsmt_simulator/drivetrain_if.h"

#include "interfaces/screen_model_if.h"

#include <Box2D/Box2D.h>

typedef uint64_t robot_id;

class Robot : public QObject
{
    Q_OBJECT

    b2Shape* _body;
    QVector<b2Shape*> _model;

public:
    Robot(b2Shape* body, DriveTrain_If* dt, QVector<Sensor_If*> sensors, QObject* parent = nullptr);

    const b2Shape* getRobotBody();
    const QVector<b2Shape*>& getRobotModel();

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

class RobotSensorsScreenModel : public ScreenModel_If
{
public:
    RobotSensorsScreenModel(Robot* robot){}

    QVector<b2Shape*> getModel(){}
    void getTransform(double& x, double& y, double& theta){}

    void setModel(QVector<b2Shape*> newModel){}
    void setTransform(double x, double y, double theta){}
};

class RobotBaseScreenModel : public ScreenModel_If
{
public:
    RobotBaseScreenModel(Robot* robot){}

    QVector<b2Shape*> getModel(){}
    void getTransform(double& x, double& y, double& theta){}

    void setModel(QVector<b2Shape*> newModel){}
    void setTransform(double x, double y, double theta){}
};

#endif // ROBOT_H
