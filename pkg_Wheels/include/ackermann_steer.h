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

class Ackermann_Steer : public WorldObjectComponent
{
    Q_OBJECT

    bool _connected = false;

    std::shared_ptr<rclcpp::Node> _rosNode;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _receiveChannel;

    Property _inputChannel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Input channel for turn angle"), "");

    Property _wradius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Wheel radius (meters)"), QVariant(0.5),
                                    &Property::abs_double_validator);

    Property _wwidth = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "Wheel width (meters)"), QVariant(0.2),
                                  &Property::abs_double_validator);

    Property _l1 = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                              "Axle length (meters)"), QVariant(1.0),
                              &Property::abs_double_validator);

    Property _l2 = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                              "Vehicle length (meters)"), QVariant(1.0),
                              &Property::abs_double_validator);

    Property _density = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE, "Density of the wheels"),
                                 QVariant(1.0), &Property::abs_double_validator);

    Property _steerAngle = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Angle of steering"),
                                 QVariant(0.0), &Property::double_validator);

#define pview(a) QSharedPointer<PropertyView>::create(a)
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"channels/input_angle", pview(&_inputChannel)},
        {"wheel_radius", pview(&_wradius)},
        {"wheel_width", pview(&_wwidth)},
        {"axle_length", pview(&_l1)},
        {"vehicle_length", pview(&_l2)},
        {"density", pview(&_density)},
        {"steer_angle", pview(&_steerAngle)}
    };
 #undef pview

    object_id _objectId;

    Model* _wheelModel = nullptr;
    Model* _lWheelModel = nullptr;
    Model* _rWheelModel = nullptr;
    Model* _debugModel = nullptr;

    b2World* _world = nullptr;
    b2Body* _anchor = nullptr;
    b2Body* _lWheelBody = nullptr, *_rWheelBody = nullptr, *_cBody = nullptr;
    b2Fixture* _lWheelFix = nullptr, *_rWheelFix = nullptr, *_cFix = nullptr;
    b2Joint* _lRevJoint = nullptr, *_rRevJoint = nullptr, *_cJoint = nullptr;

public:
    Ackermann_Steer(QObject* parent=nullptr);

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

    QString getPluginName(){ return "org.sdsmt.sim.2d.worldObjectComponent.defaults.ackermann"; }

signals:
    void _receiveMessage(const std_msgs::msg::Float32::SharedPtr data);

private slots:
    void _processMessage(const std_msgs::msg::Float32::SharedPtr data);
    void _refreshChannel(QVariant);
    void _attachWheelFixture();
    void _buildModels();
    void _jointWheels();

public slots:
    //Connects to all ROS topics
    virtual void connectChannels();

    //Disconnects all ROS topics
    virtual void disconnectChannels();

    virtual void _worldTicked(const double);
};

#endif // FLOATER_DRIVETRAIN_H
