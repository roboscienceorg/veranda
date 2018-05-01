//! \file
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <sdsmt_simulator/world_object_component.h>
#include <Box2D/Box2D.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

#include "defines.h"

/*!
 * \brief World Object Component modeling the Ackermann steering constraint
 * This component creates two wheels which can be steered together and follow
 * the Ackermann steering rule
 *
 * The component listens on a ROS channel for a Float32 message. This
 * value is interpreted as the target angle to steer at (radians). The value
 * is clampted to the range +/-PI/2
 */
class Ackermann_Steer : public WorldObjectComponent
{
    Q_OBJECT

    //! Flag indicating if ROS channels are connected
    bool _connected = false;

    //! Reference to ROS node for process
    std::shared_ptr<rclcpp::Node> _rosNode;

    //! The ROS Subscriptiong channel for steering angles
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _receiveChannel;

    //! Property: The name of the ROS subscription channel
    Property _inputChannel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Input channel for turn angle"), "");

    //! Property: The radius of the wheels (meters)
    Property _wradius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Wheel radius (meters)"), QVariant(0.5),
                                    &Property::abs_double_validator);

    //! Property: The width of the wheels (meters)
    Property _wwidth = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "Wheel width (meters)"), QVariant(0.2),
                                  &Property::abs_double_validator);

    //! Property: Half the length of the axle connecting the wheels (meters)
    Property _l1 = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                              "Axle length (meters)"), QVariant(1.0),
                              &Property::abs_double_validator);

    //! Property: Distance from the front axle to the back axle (meters)
    Property _l2 = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                              "Vehicle length (meters)"), QVariant(1.0),
                              &Property::abs_double_validator);

    //! Property: Density of the wheels (Use this to tune how much force they produce)
    Property _density = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE, "Density of the wheels"),
                                 QVariant(1.0), &Property::abs_double_validator);

    //! Property: The current angle being steered to
    Property _steerAngle = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Angle of steering"),
                                 QVariant(0.0), &Property::double_validator);

#define pview(a) QSharedPointer<PropertyView>::create(a)
    //! Mapping of ids to properties for the component
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

    //! Record of the object id this component is part of
    object_id _objectId;

    //! Model for the axle
    Model* _wheelModel = nullptr;

    //! Model for the left wheel
    Model* _lWheelModel = nullptr;

    //! Model for the right wheel
    Model* _rWheelModel = nullptr;

    //! Model for debugging with extra images
    Model* _debugModel = nullptr;

    //! Reference to the b2World
    b2World* _world = nullptr;

    //! Reference to object anchor body
    b2Body* _anchor = nullptr;

    //! Body for left wheel
    b2Body* _lWheelBody = nullptr;

    //! Body for right wheel
    b2Body* _rWheelBody = nullptr;

    //! Body for axle center
    b2Body* _cBody = nullptr;

    //! Fixture for left wheel
    b2Fixture* _lWheelFix = nullptr;

    //! Fixture for right wheel
    b2Fixture* _rWheelFix = nullptr;

    //! Fixture for axle
    b2Fixture* _cFix = nullptr;

    //! Joint connecting left wheel
    b2Joint* _lRevJoint = nullptr;

    //! Joint connecting right wheel
    b2Joint* _rRevJoint = nullptr;

    //! Joint connecting axle
    b2Joint* _cJoint = nullptr;

public:
    /*!
     * \brief Constructs a new Ackermann steer component
     * \param[in] parent QObject parent
     */
    Ackermann_Steer(QObject* parent=nullptr);

    /*!
     * \brief Creates a copy Ackermann Steer componenet
     * \param[in] newParent QObject parent of new component
     * \return A newly constructcted Ackermann Steer component
     */
    WorldObjectComponent* _clone(QObject *newParent);

    /*!
     * \brief Getter for properties unique to Ackermann Steer
     * \return Mapping of the properties unique to Ackermann Steer
     */
    virtual QMap<QString, QSharedPointer<PropertyView>> _getProperties(){
        return _properties;
    }

    /*!
     * \brief Getter to check if the component uses ROS channels
     * \return true
     */
    bool usesChannels(){
        return true;
    }

    /*!
     * \brief Gives the component the chance to create physics bodies for itself
     * \param[in] world The box2D world in use
     * \param[in] oId The id of the object this component is part of
     * \param[in] anchor The anchor body to joint to
     */
    void generateBodies(b2World* world, object_id oId, b2Body* anchor);

    //! The component removes itself from the b2World it was last added to
    void clearBodies();

    //! Sets the ROS node to be used for subscriptions
    void setROSNode(std::shared_ptr<rclcpp::Node> node);

    /*!
     * \brief Returns the plugin IID
     * \return "org.sdsmt.sim.2d.worldObjectComponent.defaults.ackermann"
     */
    QString getPluginName(){ return ACKERMANN_IID; }

signals:
    /*!
     * \brief Signals that a ROS message has been recieved
     * \param[in] data The Float32 data message
     */
    void _receiveMessage(const std_msgs::msg::Float32::SharedPtr data);

private slots:
    /*!
     * \brief Adjusts steering angles based on input messages
     * \param[in] data The input steering message
     */
    void _processMessage(const std_msgs::msg::Float32::SharedPtr data);

    //! Closes and re-opens the ROS channel
    void _refreshChannel();

    //! Rebuilds and attaches wheel fixtures
    void _attachWheelFixture();

    //! Clears and rebuilds the shapes in the Models for the component
    void _buildModels();

    //! Destroys and re-creates the joints between the wheels and anchor body
    void _jointWheels();

public slots:
    //! Creates the ROS subscription channel
    virtual void connectChannels();

    //! Destroys the ROS subscription channel
    virtual void disconnectChannels();

    //! Applies wheel slip/slide constraints each tick of the physics
    virtual void _worldTicked(const double);
};

