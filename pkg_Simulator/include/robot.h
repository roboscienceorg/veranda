#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>

#include "sdsmt_simulator/sensor_if.h"
#include "sdsmt_simulator/drivetrain_if.h"

#include "interfaces/screen_model_if.h"

#include <Box2D/Box2D.h>
#include <QMap>
#include <QMutex>

typedef uint64_t robot_id;

class Robot : public PropertyObject_If
{
    Q_OBJECT

    double _x=0, _y=0, _theta=0;

    b2Body* _bodyPhysics;
    QVector<b2Shape*> _model;

    DriveTrain_If* _drivetrain;
    QVector<Sensor_If*> _sensors;

    QMap<QString, PropertyView> _properties;

public:
    Robot(QVector<b2Shape*> body, DriveTrain_If* dt, QVector<Sensor_If*> sensors = QVector<Sensor_If*>(), QObject* parent = nullptr);

    void setPhysicsBody(b2Body* body);

    QMap<QString, PropertyView>& getAllProperties(){ return _properties; }
    const QVector<b2Shape*> getRobotModel(){return _model;}

    virtual QString propertyGroupName(){return "";}
public slots:
    //Tells the robot to connect all its ROS topics
    void connectToROS();

    //Tells the robot to disconnect all its ROS topics
    void disconnectFromROS();

    //Tells the robot that the world has updated
    void worldTicked(const double t, const b2World *world);

    void targetVelocity(double x, double y, double theta);

signals:
    void _newPosition(double x, double y, double theta);
};

class RobotSensorsScreenModel : public ScreenModel_If
{
    Q_OBJECT
public:
    RobotSensorsScreenModel(Robot* robot){}

    QVector<b2Shape*> getModel(){return QVector<b2Shape*>{};}
    void getTransform(double& x, double& y, double& theta){}

    void setModel(QVector<b2Shape*> newModel){}
    void setTransform(double x, double y, double theta){}
};

class RobotBaseScreenModel : public ScreenModel_If
{
    Q_OBJECT

    QVector<b2Shape*> model;
    b2BlockAllocator alloc;

    double _x, _y, _theta;
public:
    RobotBaseScreenModel(Robot* robot)
    {
        model = robot->getRobotModel();
        connect(robot, &Robot::_newPosition, this, &RobotBaseScreenModel::robotMoved);
    }

    QVector<b2Shape*> getModel(){return model;}
    void getTransform(double& x, double& y, double& theta)
    {
        x = _x;
        y = _y;
        theta = _theta;
    }

    void setModel(QVector<b2Shape*> newModel){}
    void setTransform(double x, double y, double theta){}

private slots:
    void robotMoved(double x, double y, double theta)
    {
        double dx = x - _x;
        double dy = y - _y;
        double dt = theta - _theta;

        _x = x;
        _y = y;
        _theta = theta;

        transformChanged(this, dx, dy, dt);
    }
};

#endif // ROBOT_H
