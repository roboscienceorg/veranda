#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>

#include "sdsmt_simulator/sensor_if.h"
#include "sdsmt_simulator/drivetrain_if.h"

#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/cloneshape.h"
#include "interfaces/old_world_object_if.h"

#include <Box2D/Box2D.h>
#include <QMap>
#include <QDebug>

class Robot : public depracatedWorldObject_If
{
    Q_OBJECT

    constexpr static double PI = 3.14159265359;
    constexpr static double RAD2DEG = 360.0/(2*PI);
    constexpr static double DEG2RAD = 1.0/RAD2DEG;

    double _x=0, _y=0, _theta=0;
    double _x0=0, _y0=0, _theta0=0;
    double _lastTick = 0;

    uint64_t _bodiesRequired = 1;

    b2Body* _mainBody = nullptr;
    Model* _model = nullptr;
    QVector<Model*> _allModels;
    QVector<b2Body*> _allBodies;
    QVector<b2Shape*> _mainShapes;
    double totalMass;

    DriveTrain_If* _drivetrain;
    QVector<Sensor_If*> _sensors;

    QMap<QString, PropertyView> _properties;

    b2JointDef* _defineJoint(b2Body* a, b2Body* b);

public:
    Robot(QVector<b2Shape*> body, DriveTrain_If* dt, QVector<Sensor_If*> sensors = QVector<Sensor_If*>(), QObject* parent = nullptr);
    ~Robot();

    void setOrientation(double x0, double y0, double theta0);

    //Virtual clone
    virtual depracatedWorldObject_If* clone(QObject* newParent=nullptr);

    //Tells the physics engine how many bodies are needed
    virtual uint64_t staticBodiesRequired(){ return 0; }
    virtual uint64_t dynamicBodiesRequired(){ return _bodiesRequired; }

    //Physics engine assigns bodies
    virtual void setStaticBodies(QVector<b2Body*>&) {}
    virtual QVector<b2JointDef*> setDynamicBodies(QVector<b2Body*>& bodies);

    virtual void clearStaticBodies(){}
    virtual void clearDynamicBodies();

    QMap<QString, PropertyView>& getAllProperties(){ return _properties; }
    virtual QString propertyGroupName(){return "";}

    //Interface for world view to draw
    virtual QVector<Model*> getModels(){return _allModels;}

public slots:
    virtual void connectChannels();
    virtual void disconnectChannels();

    //Tells the robot that the world has updated
    virtual void worldTicked(const b2World* world, const double& t);

private slots:
    void targetVelocity(double x, double y, double theta);
    void computeMass();
};

#endif // ROBOT_H
