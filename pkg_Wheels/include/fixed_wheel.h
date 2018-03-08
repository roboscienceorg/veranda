#ifndef FIXED_WHEEL_H
#define FIXED_WHEEL_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <sdsmt_simulator/world_object_component.h>
#include <Box2D/Box2D.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

class Fixed_Wheel : public WorldObjectComponent
{
    Q_OBJECT

    bool _connected = false;

    std::shared_ptr<rclcpp::Node> _rosNode;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _receiveChannel;

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

    Property _density = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE, "Density of the wheel"),
                                 QVariant(1.0), &Property::abs_double_validator);

 #define pview(a) QSharedPointer<PropertyView>::create(a)
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"channels/input_speed", pview(&_inputChannel)},
        {"wheel_radius", pview(&_radius)},
        {"wheel_width", pview(&_width)},
        {"is_driven", pview(&_driven)},
        {"density", pview(&_density)}
    };
#undef pview

    object_id _objectId;

    Model* _wheelModel = nullptr;
    QVector<b2Shape*> _wheelShapes;
    double _objectMass;

    b2Body* _wheelBody = nullptr;
    b2Fixture* _wheelFix = nullptr;
    b2Joint* _weldJoint = nullptr;

    b2World* _world = nullptr;

    //Data published
    double _targetAngularVelocity = 0;

public:
    Fixed_Wheel(QObject* parent=nullptr);

    WorldObjectComponent* _clone(QObject *newParent);

    virtual QMap<QString, QSharedPointer<PropertyView>> _getProperties(){
        return _properties;
    }

    bool usesChannels(){
        return true;
    }

    void generateBodies(b2World* world, object_id oId, b2Body* anchor);
    void clearBodies();

    void setROSNode(std::shared_ptr<rclcpp::Node> node);

signals:
    void _receiveMessage(const std_msgs::msg::Float32::SharedPtr data);

private slots:
    void _processMessage(const std_msgs::msg::Float32::SharedPtr data);
    void _refreshChannel(QVariant);
    void _attachWheelFixture();
    void _buildModels();

public slots:
    //Connects to all ROS topics
    virtual void connectChannels();

    //Disconnects all ROS topics
    virtual void disconnectChannels();

    virtual void _worldTicked(const double);
};

#endif // FLOATER_DRIVETRAIN_H
