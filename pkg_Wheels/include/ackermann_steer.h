#ifndef FIXED_WHEEL_H
#define FIXED_WHEEL_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <sdsmt_simulator/world_object_component_if.h>
#include <Box2D/Box2D.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

class Ackermann_Steer : public WorldObjectComponent_If
{
    Q_OBJECT

    bool _connected = false;

    std::shared_ptr<rclcpp::Node> _rosNode;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _receiveChannel;

    Property _xLocal = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "X location of component within object"),
                                QVariant(0.0), &Property::double_validator);

    Property _yLocal = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Y location of component within object"),
                                QVariant(0.0), &Property::double_validator);

    Property _thetaLocal = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Angle of component within object"),
                                QVariant(0.0), &Property::angle_validator);

    Property _inputChannel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Input channel for turn angle"), "");

    Property _wradius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Wheel radius (meters)"), QVariant(0.1),
                                    &Property::abs_double_validator);

    Property _wwidth = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "Wheel width (meters)"), QVariant(0.0),
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

    QMap<QString, PropertyView> _properties{
        {"channels/input_angle", &_inputChannel},
        {"x_local", &_xLocal},
        {"y_local", &_yLocal},
        {"theta_local", &_thetaLocal},
        {"wheel_radius", &_wradius},
        {"wheel_width", &_wwidth},
        {"axle_length", &_l1},
        {"vehicle_length", &_l2},
        {"density", &_density},
        {"steer_angle", &_steerAngle}
    };

    object_id _objectId;

    Model* _wheelModel = nullptr;
    Model* _lWheelModel = nullptr;
    Model* _rWheelModel = nullptr;
    Model* _debugModel = nullptr;

    double _objectMass;

    b2World* _world = nullptr;
    b2Body* _anchor = nullptr;
    b2Body* _lWheelBody = nullptr, *_rWheelBody = nullptr, *_cBody = nullptr;
    b2Fixture* _lWheelFix = nullptr, *_rWheelFix = nullptr, *_cFix = nullptr;
    b2Joint* _lRevJoint = nullptr, *_rRevJoint = nullptr, *_cJoint = nullptr;

public:
    Ackermann_Steer(QObject* parent=nullptr);

    WorldObjectComponent_If* clone(QObject *newParent);

    virtual QMap<QString, PropertyView>& getProperties(){
        return _properties;
    }

    virtual QString getPropertyGroup(){
        return "Ackermann Steer";
    }

    QVector<Model*> getModels(){
        return {_wheelModel};
    }

    bool usesChannels(){
        return true;
    }

    void generateBodies(b2World* world, object_id oId, b2Body* anchor);
    void clearBodies(b2World *world);

    void setROSNode(std::shared_ptr<rclcpp::Node> node);

signals:
    void _receiveMessage(const std_msgs::msg::Float32::SharedPtr data);

private slots:
    void _processMessage(const std_msgs::msg::Float32::SharedPtr data);
    void _refreshChannel(QVariant);
    void _attachWheelFixture();
    void _buildModels();
    void _jointWheels();
    void _updateModelLocations();

public slots:
    //Connects to all ROS topics
    virtual void connectChannels();

    //Disconnects all ROS topics
    virtual void disconnectChannels();

    virtual void worldTicked(const b2World*, const double);

    virtual void setObjectMass(double mass)
    {
        _objectMass = mass;
    }
};

#endif // FLOATER_DRIVETRAIN_H
