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

    QString _inputChannel;
    bool _connected = false;

    ros::NodeHandle _rosNode;
    ros::Subscriber _receiveChannel;

    Property x_local = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "X location of component within object"),
                                QVariant(0.0), &Property::double_validator);

    Property y_local = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Y location of component within object"),
                                QVariant(0.0), &Property::double_validator);

    Property theta_local = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Angle of component within object"),
                                QVariant(0.0), &Property::angle_validator);

    Property input_channel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Input channel for drive speed"), "");

    Property radius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Wheel radius (meters)"), QVariant(0.1),
                                    &Property::abs_double_validator);

    Property width = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "Wheel width (meters)"), QVariant(0.0),
                                  &Property::abs_double_validator);

    Property driven = Property(PropertyInfo(false, false, PropertyInfo::BOOL, "Whether or not the wheel is driven"),
                               QVariant(false), &Property::bool_validator);

    Property max_rps = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                               "Maximum radians per second the wheel can spin"),
                                               QVariant(1.0), &Property::abs_double_validator);

    QMap<QString, PropertyView> _properties{
        {"channels/input_speed", &input_channel},
        {"x_local", &x_local},
        {"y_local", &y_local},
        {"theta_local", &theta_local},
        {"wheel_radius", &radius},
        {"wheel_width", &width},
        {"is_driven", &driven},
        {"max_rps", &max_rps}
    };

    object_id objectId;

    Model* wheel_model = nullptr;
    QVector<b2Shape*> wheel_shapes;

    b2Body* wheelBody = nullptr;
    b2Fixture* wheelFix = nullptr;
    b2Joint* weldJoint = nullptr;
    b2Vec2 localWheelFrontUnit;
    b2Vec2 localWheelRightUnit;

    //Data published
    double curr_percent = 0;
    double curr_rps = 0;

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
        return {wheel_model};
    }

    bool usesChannels(){
        return true;
    }

    void generateBodies(b2World* world, object_id oId, b2Body* anchor);
    void clearBodies(b2World *world);

signals:
    void _receiveMessage(std_msgs::Float32 data);

private slots:
    void _processMessage(std_msgs::Float32 data);
    void _refreshChannel(QVariant);
    void _attachWheelFixture();
    void _buildModels();
    void _updateForce()
    {
        curr_rps = curr_percent*max_rps.get().toDouble();
    }

public slots:
    //Connects to all ROS topics
    virtual void connectChannels();

    //Disconnects all ROS topics
    virtual void disconnectChannels();

    virtual void worldTicked(const b2World*, const double);
};

#endif // FLOATER_DRIVETRAIN_H
