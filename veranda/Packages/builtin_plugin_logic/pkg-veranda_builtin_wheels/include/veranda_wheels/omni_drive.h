//! \file
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include <veranda_core/world_object_component.h>
#include <Box2D/Box2D.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

#include <veranda_core/filter.h>
#include <veranda_core/dllapi.h>

/*!
 * \brief World Object Component modeling a ball that can roll wherever you want.
 * The drive is commanded with target x, y, and theta velocities in the global reference frame
 */
class veranda_API Omni_Drive : public WorldObjectComponent
{
    Q_OBJECT

    //! Reference to ROS node for process
    std::shared_ptr<rclcpp::Node> _rosNode;

    //! The ROS Subscriptiong channel for velocity targets
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr _receiveChannel;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr _publishChannel;

    //! Property: The name of the ROS subscription channel
    Property _inputChannel = Property(PropertyInfo(false, true, false, PropertyInfo::STRING,
                                                    "Input channel for travel velocities"), "");

    //! Property: The name of the ROS subscription channel
    Property _outputChannel = Property(PropertyInfo(false, true, false, PropertyInfo::STRING,
                                                    "Output channel for travel velocities"), "");

    //! Property: The radius of the ball (meters)
    Property _radius = Property(PropertyInfo(true, true, false, PropertyInfo::DOUBLE,
                                    "Ball radius (meters)"), QVariant(0.5),
                                    &Property::abs_double_validator);

    //! Property: Flag for if the ball can be driven or not
    Property _driven = Property(PropertyInfo(false, true, false, PropertyInfo::BOOL, "Whether or not the ball is driven"),
                               QVariant(false), &Property::bool_validator);

    //! Property: Density of the ball (Use this to tune how much force it produces)
    Property _density = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Density of the ball"),
                                 QVariant(1.0), &Property::abs_double_validator);

    //! Property: Noise in control (sigma)
    Property in_noise_sigma = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in control (sigma)"),
                                QVariant(0.0), &Property::abs_double_validator);

    //! Property: Noise in control (mu)
    Property in_noise_mu = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in control (mu)"),
                                QVariant(0.0), &Property::double_validator);

    //! Property: Noise is reported velocities (sigma)
    Property out_noise_sigma = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in reported (sigma)"),
                                QVariant(0.0), &Property::abs_double_validator);

    //! Property: Noise in control (mu)
    Property out_noise_mu = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in reported (mu)"),
                                QVariant(0.0), &Property::double_validator);

    //! Property: Rate of publishing messages
    Property pub_rate = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Reporting rate (hz)"),
                                 QVariant(10), &Property::abs_double_validator);

    typedef std::mt19937_64 reng_type;

    NormalFilter<reng_type> drive_filter;
    NormalFilter<reng_type> report_filter;

 #define pview(a) QSharedPointer<PropertyView>::create(a)
    //! Mapping of ids to properties for the component
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"channels/input_speeds", pview(&_inputChannel)},
        {"channels/reported_speeds", pview(&_outputChannel)},
        {"ball_radius", pview(&_radius)},
        {"is_driven", pview(&_driven)},
        {"density", pview(&_density)},
        {"control_noise/mu", pview(&in_noise_mu)},
        {"control_noise/sigma", pview(&in_noise_sigma)},
        {"reported_noise/mu", pview(&out_noise_mu)},
        {"reported_noise/sigma", pview(&out_noise_sigma)},
        {"report_rate", pview(&pub_rate)}
    };
#undef pview

    //! Record of the object id this component is part of
    object_id _objectId;

    //! Model for the left
    Model* _ballModel = nullptr;

    //! Set of the shapes drawn to represent the wheel
    QVector<b2Shape*> _ballShapes;

    //! Body for the wheel
    b2Body* _ballBody = nullptr;

    //! Fixture for the wheel
    b2Fixture* _ballFix = nullptr;

    //! Joint connecting the wheel to the anchor
    b2Joint* _weldJoint = nullptr;

    //! Reference to the b2World
    b2World* _world = nullptr;

    //! Last recieved target angular velocity
    double _targetAngularVelocity = 0;

    //! Last recieved target x velocity
    double _targetXVelocity = 0;

    //! Last recieved target y velocity
    double _targetYVelocity = 0;

    //! Time since last message publish
    double _timeSincePublish = 0;

    //! IID of plugin that generated this object
    const QString _pluginIID;

public:
    /*!
     * \brief Constructs a new wheel component
     * \param[in] parent QObject parent
     */
    Omni_Drive(const QString& pluginIID, QObject* parent=nullptr);
    virtual ~Omni_Drive() override {}

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
     * \param[in] data The Pose2D data message
     */
    void _receiveMessage(const geometry_msgs::msg::Pose2D::SharedPtr data);

private slots:
    /*!
     * \brief Stores the target velocities read from ROS
     * \param data The new velocity
     */
    void _processMessage(const geometry_msgs::msg::Pose2D::SharedPtr data);

    //! Closes and re-opens the ROS channel
    void _refreshChannel(QVariant);

    //! Rebuilds and attaches wheel fixture
    void _attachFixture();

    //! Clears and rebuilds the shapes in the Model for the component
    void _buildModels();

public slots:
    //! Creates the ROS subscription channel
    virtual void _connectChannels() override;

    //! Destroys the ROS subscription channel
    virtual void _disconnectChannels() override;

    //! Applies slip/slide constraints each tick of the physics
    virtual void _worldTicked(const double dt) override;
};
