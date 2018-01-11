#ifndef FIXED_WHEEL_H
#define FIXED_WHEEL_H

#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <sdsmt_simulator/world_object_component_if.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

class Fixed_Wheel : public WorldObjectComponent_If
{
    Q_OBJECT

    constexpr static double PI = 3.14159265359;
    constexpr static double RAD2DEG = 360.0/(2*PI);
    constexpr static double DEG2RAD = 1.0/RAD2DEG;

    bool _connected = false;

    ros::NodeHandle _rosNode;
    ros::Subscriber _receiveChannel;

    Property _xLocal = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "X location of component within object"),
                                QVariant(0.0), &Property::double_validator);

    Property _yLocal = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Y location of component within object"),
                                QVariant(0.0), &Property::double_validator);

    Property _thetaLocal = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Angle of component within object"),
                                QVariant(0.0), &Property::angle_validator);

    Property _inputChannel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Input channel for drive speed"), "");

    Property _radius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Wheel radius (meters)"), QVariant(0.1),
                                    &Property::abs_double_validator);

    Property _width = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "Wheel width (meters)"), QVariant(0.0),
                                  &Property::abs_double_validator);

    Property _driven = Property(PropertyInfo(false, false, PropertyInfo::BOOL, "Whether or not the wheel is driven"),
                               QVariant(false), &Property::bool_validator);

    QMap<QString, PropertyView> _properties{
        {"channels/input_speed", &_inputChannel},
        {"x_local", &_xLocal},
        {"y_local", &_yLocal},
        {"theta_local", &_thetaLocal},
        {"wheel_radius", &_radius},
        {"wheel_width", &_width},
        {"is_driven", &_driven}
    };

    object_id _objectId;

    Model* _wheelModel = nullptr;
    QVector<b2Shape*> _wheelShapes;

    b2Body* _wheelBody = nullptr;
    b2Fixture* _wheelFix = nullptr;
    b2Joint* _weldJoint = nullptr;
    b2Vec2 _localWheelFrontUnit;
    b2Vec2 _localWheelRightUnit;

    //Data published
    double _targetAngularVelocity;

public:
    Fixed_Wheel(QObject* parent=nullptr);

    WorldObjectComponent_If* clone(QObject *newParent);

    virtual QMap<QString, PropertyView> getProperties(){
        return _properties;
    }

    virtual QString getPropertyGroup(){
        return "Fixed Wheel";
    }

    QVector<Model*> getModels(){
        return {_wheelModel};
    }

    bool usesChannels(){
        return true;
    }

    QVector<b2Body *> generateBodies(b2World* world, object_id oId, b2Body* anchor);
    void clearBodies(b2World *world);

signals:
    void _receiveMessage(std_msgs::Float32 data);

private slots:
    void _processMessage(std_msgs::Float32 data);
    void _refreshChannel(QVariant);
    void _attachWheelFixture();
    void _buildModels();

public slots:
    //Connects to all ROS topics
    virtual void connectChannels();

    //Disconnects all ROS topics
    virtual void disconnectChannels();

    virtual void worldTicked(const b2World*, const double);
};

#endif // FLOATER_DRIVETRAIN_H
