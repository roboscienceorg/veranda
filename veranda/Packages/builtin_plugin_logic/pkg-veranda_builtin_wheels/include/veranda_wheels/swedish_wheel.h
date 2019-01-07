//! \file
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <veranda_core/api/world_object_component.h>
#include <Box2D/Box2D.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

#include <veranda_core/api/filter.h>
#include <veranda_wheels/encoder.h>
#include <veranda_wheels/dllapi.h>


/*!
 * \brief World Object Component modeling a single fixed wheel
 * This component creates a single wheel which obeys the no-slide constraint, and
 * maybe driving and will obey the no-slip constraint
 *
 * The component listens on a ROS channel for a Float32 message. This
 * value is interpreted as the speed to drive the wheel (rad/s)
 */
class VERANDA_WHEELS_DLL Swedish_Wheel : public WorldObjectComponent
{
    Q_OBJECT

    static QVariant half_angle_validator(const QVariant& oldAngle, const QVariant& newAngle)
    {
        double newDouble = newAngle.toDouble();
        if(newDouble >= -90 && newDouble <= 90)
            return newDouble;
        return oldAngle;
    }

    //! Reference to ROS node for process
    std::shared_ptr<rclcpp::Node> _rosNode;

    Encoder wheelEncoder;

    //! The ROS Subscriptiong channel for steering angles
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _receiveChannel;

    //! Property: The name of the ROS subscription channel
    Property _inputChannel = Property(PropertyInfo(false, true, false, PropertyInfo::STRING,
                                                    "Input channel for drive speed"), "");

    //! Property: The radius of the wheel (meters)
    Property _radius = Property(PropertyInfo(true, true, false, PropertyInfo::DOUBLE,
                                    "Wheel radius (meters)"), QVariant(0.5),
                                    &Property::abs_double_validator);

    //! Property: The width of the wheel (meters)
    Property _width = Property(PropertyInfo(true, true, false, PropertyInfo::DOUBLE,
                                  "Wheel width (meters)"), QVariant(0.2),
                                  &Property::abs_double_validator);

    //! Property: The width of the wheel (meters)
    Property _rollerAngle = Property(PropertyInfo(true, true, false, PropertyInfo::DOUBLE,
                                        "Roller Angle (-90-90 degrees)"), QVariant(0),
                                        &half_angle_validator);

    //! Property: Flag for if the wheel can be driven or not
    Property _driven = Property(PropertyInfo(false, true, false, PropertyInfo::BOOL, "Whether or not the wheel is driven"),
                               QVariant(false), &Property::bool_validator);

    //! Property: Density of the wheel (Use this to tune how much force it produces)
    Property _density = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Density of the wheel"),
                                 QVariant(1.0), &Property::abs_double_validator);

    //! Property: Noise is control (sigma)
    Property noise_sigma = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in control (sigma)"),
                                QVariant(0.0), &Property::abs_double_validator);

    //! Property: Noise in control (mu)
    Property noise_mu = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in control (mu)"),
                                QVariant(0.0), &Property::double_validator);

    NormalFilter<> drive_filter;

 #define pview(a) QSharedPointer<PropertyView>::create(a)
    //! Mapping of ids to properties for the component
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"channels/input_speed", pview(&_inputChannel)},
        {"wheel_radius", pview(&_radius)},
        {"wheel_width", pview(&_width)},
        {"roller_angle", pview(&_rollerAngle)},
        {"is_driven", pview(&_driven)},
        {"density", pview(&_density)},
        {"control_noise/mu", pview(&noise_mu)},
        {"control_noise/sigma", pview(&noise_sigma)}
    };
#undef pview
    //! Record of the object id this component is part of
    object_id _objectId;

    //! Model for the left
    Model* _wheelModel = nullptr;

    //! Set of the shapes drawn to represent the wheel
    QVector<b2Shape*> _wheelShapes;

    //! Body for the wheel
    b2Body* _wheelBody = nullptr;

    //! Fixture for the wheel
    b2Fixture* _wheelFix = nullptr;

    //! Joint connecting the wheel to the anchor
    b2Joint* _weldJoint = nullptr;

    //! Reference to the b2World
    b2World* _world = nullptr;

    //! Last recieved target angular velocity
    double _targetAngularVelocity = 0;

    //! IID of plugin that generated this object
    const QString _pluginIID;

public:
    /*!
     * \brief Constructs a new wheel component
     * \param[in] parent QObject parent
     */
    Swedish_Wheel(const QString& pluginIID, QObject* parent=nullptr);
    virtual ~Swedish_Wheel() override {}

    /*!
     * \brief Creates a copy wheel componenet
     * \param[in] newParent QObject parent of new component
     * \return A newly constructcted wheel component
     */
    WorldObjectComponent* _clone(QObject *newParent) override;

    /*!
     * \brief Getter for properties unique to Ackermann Steer
     * \return Mapping of the properties unique to Ackermann Steer
     */
    virtual QMap<QString, QSharedPointer<PropertyView>> _getProperties() override{
        return _properties;
    }

    /*!
     * \brief Getter to check if the component uses ROS channels
     * \return true
     */
    bool usesChannels() override{
        return true;
    }

    /*!
     * \brief Gives the component the chance to create physics bodies for itself
     * \param[in] world The box2D world in use
     * \param[in] oId The id of the object this component is part of
     * \param[in] anchor The anchor body to joint to
     */
    void _generateBodies(b2World* world, object_id oId, b2Body* anchor) override;

    //! The component removes itself from the b2World it was last added to
    void _clearBodies() override;

    //! Sets the ROS node to be used for subscriptions
    void _setROSNode(std::shared_ptr<rclcpp::Node> node) override;

    /*!
     * \brief Returns the plugin IID
     * \return The IID of the plugin providing this component
     */
    QString getPluginName() override { return _pluginIID; }

signals:
    /*!
     * \brief Signals that a ROS message has been recieved
     * \param[in] data The Float32 data message
     */
    void _receiveMessage(const std_msgs::msg::Float32::SharedPtr data);

private slots:
    /*!
     * \brief Stores the target angular velocity read from ROS
     * \param data The new velocity
     */
    void _processMessage(const std_msgs::msg::Float32::SharedPtr data);

    //! Closes and re-opens the ROS channel
    void _refreshChannel(QVariant);

    //! Rebuilds and attaches wheel fixture
    void _attachWheelFixture();

    //! Clears and rebuilds the shapes in the Model for the component
    void _buildModels();

public slots:
    //! Creates the ROS subscription channel
    virtual void _connectChannels() override;

    //! Destroys the ROS subscription channel
    virtual void _disconnectChannels() override;

    //! Applies slip/slide constraints each tick of the physics
    virtual void _worldTicked(const double) override;
};
