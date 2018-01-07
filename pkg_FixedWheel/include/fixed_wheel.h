#ifndef FIXED_WHEEL_H
#define FIXED_WHEEL_H

#include "ros/ros.h"
#include "std_msgs/ByteMultiArray.h"

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

    Property input_channel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Input channel for drive speed"), "");

    Property radius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Wheel radius (meters)"), QVariant(0.1),
                                    &Property::double_validator);

    Property width = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "Wheel width (meters)"), QVariant(0.0),
                                  &Property::double_validator);

    Property driven = Property(PropertyInfo(false, false, PropertyInfo::BOOL, "Whether or not the wheel is driven"),
                               QVariant(false), &Property::bool_validator);

    Property max_force = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                               "Maximum linear force the wheel can generate (Newtons?)"),
                                               QVariant(1.0), Property::double_validator);

    QMap<QString, PropertyView> _properties{
        {"channels/input_speed", &input_channel},
        {"wheel_radius", &radius},
        {"wheel_width", &width},
        {"is_driven", &driven},
        {"max_force", &max_force}
    };

    Model* wheel_model = nullptr;

    b2Body* wheelBody = nullptr;
    b2Fixture* wheelFix = nullptr;

    //Data published
    double curr_speed;

    //Track of what's shown
    QSet<int> active_touches;

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

private slots:
    QVector<b2JointDef*> setDynamicBodies(QVector<b2Body *> & bodies);
    void _channelChanged(QVariant);
    void _attachWheelFixture();
    void _buildModels();

public slots:
    //Connects to all ROS topics
    virtual void connectChannels();

    //Disconnects all ROS topics
    virtual void disconnectChannels();

    virtual void worldTicked(const b2World*, const double&);
};

#endif // FLOATER_DRIVETRAIN_H
