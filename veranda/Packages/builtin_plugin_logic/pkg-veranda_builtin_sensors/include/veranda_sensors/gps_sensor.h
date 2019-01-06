//! \file
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include <veranda_core/filter.h>
#include <veranda_core/world_object_component.h>
#include <Box2D/Box2D.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>
#include <QPair>

#include <memory>
#include <limits>
#include <cmath>
#include <random>

/*!
 * \brief Component which publishes the Robot pose using the Pose2D message.
 * Because it may be useful to simulate a broken GPS, this plugin supports
 * * Cumulative drift over time
 * * Noisy readings
 * * Dropped information (Simulated by publishing nan instead of a valid value)
 *
 * Drift and noise statistics follow a normal distribution, dropped information is
 * uniform
 */
class GPS_Sensor : public WorldObjectComponent
{
    Q_OBJECT

    typedef std::mt19937_64 reng_type;

    //! Random number engine
    QSharedPointer<reng_type> _reng;

    //! Size of the gps body when drawn on screen
    constexpr static double SHAPE_RADIUS = 0.5;

    //! Id of the WorldObject the gps is part of
    object_id objectId = 0;

    //! ROS channel to publish on
    QString _outputChannel;

    //! ROS node to publish messages with
    std::shared_ptr<rclcpp::Node> _rosNode;

    //! ROS channel to publish gps messages on
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr _sendChannel;

    //! Property: ROS channel to publish on
    Property output_channel = Property(PropertyInfo(false, true, false, PropertyInfo::STRING,
                                                    "Output channel for gps messages"), "");

    //! Property: Rate of publishing messages
    Property pub_rate = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Publish rate (hz)"),
                                 QVariant(10), &Property::abs_double_validator);

    //! Property: Chance of publishing x coord
    Property x_chance = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Probability of publishing x coord"),
                                 QVariant(1.0), &Property::probability_validator);

    //! Property: Chance of publishing y coord
    Property y_chance = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Probability of publishing y coord"),
                                 QVariant(1.0), &Property::probability_validator);

    //! Property: Chance of publishing theta
    Property t_chance = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Probability of publishing theta"),
                                 QVariant(1.0), &Property::probability_validator);

    //! Property: Max drift magnitude accumulated per second in x direction (sigma)
    Property x_drift_sigma = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Max accumulated drift per second in x direction (sigma)"),
                                QVariant(0.0), &Property::abs_double_validator);

    //! Property: Max drift magnitude accumulated per second in x direction (mu)
    Property x_drift_mu = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Max accumulated drift per second in x direction (mu)"),
                                QVariant(0.0), &Property::double_validator);

    //! Property: Max drift magnitude accumulated per second in y direction (sigma)
    Property y_drift_sigma = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Max accumulated drift per second in y direction (sigma)"),
                                QVariant(0.0), &Property::abs_double_validator);

    //! Property: Max drift magnitude accumulated per second in y direction (mu)
    Property y_drift_mu = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Max accumulated drift per second in y direction (mu)"),
                                QVariant(0.0), &Property::double_validator);

    //! Property: Max drift magnitude accumulated per second in theta (sigma)
    Property t_drift_sigma = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Max accumulated drift per second in theta (sigma)"),
                                QVariant(0.0), &Property::abs_double_validator);

    //! Property: Max drift magnitude accumulated per second in theta (mu)
    Property t_drift_mu = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Max accumulated drift per second in theta (mu)"),
                                QVariant(0.0), &Property::double_validator);

    //! Property: Noise is x readings (sigma)
    Property x_noise_sigma = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in x readings (sigma)"),
                                QVariant(0.0), &Property::abs_double_validator);

    //! Property: Noise in x readings (mu)
    Property x_noise_mu = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in x readings (mu)"),
                                QVariant(0.0), &Property::double_validator);

    //! Property: Noise is x readings (sigma)
    Property y_noise_sigma = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in y readings (sigma)"),
                                QVariant(0.0), &Property::abs_double_validator);

    //! Property: Noise in x readings (mu)
    Property y_noise_mu = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in y readings (mu)"),
                                QVariant(0.0), &Property::double_validator);

    //! Property: Noise is x readings (sigma)
    Property t_noise_sigma = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in t readings (sigma)"),
                                QVariant(0.0), &Property::abs_double_validator);

    //! Property: Noise in x readings (mu)
    Property t_noise_mu = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in t readings (mu)"),
                                QVariant(0.0), &Property::double_validator);

    //! Filter for theta drift
    NormalFilter<reng_type> t_drift_filter;

    //! Filter for theta noise
    NormalFilter<reng_type> t_noise_filter;

    //! Filter for x drift
    NormalFilter<reng_type> x_drift_filter;

    //! Filter for x noise
    NormalFilter<reng_type> x_noise_filter;

    //! Filter for y drift
    NormalFilter<reng_type> y_drift_filter;

    //! Filter for y noise
    NormalFilter<reng_type> y_noise_filter;

#define pview(a) QSharedPointer<PropertyView>(new PropertyView(a))
    //! Mapping of gps propertys by their identifiers
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"channels/output_pose", pview(&output_channel)},
        {"publish_rate", pview(&pub_rate)},
        {"probabilities/x", pview(&x_chance)},
        {"probabilities/y", pview(&y_chance)},
        {"probabilities/theta", pview(&t_chance)},
        {"drift/x/sigma", pview(&x_drift_sigma)},
        {"drift/x/mu", pview(&x_drift_mu)},
        {"drift/y/sigma", pview(&y_drift_sigma)},
        {"drift/y/mu", pview(&y_drift_mu)},
        {"drift/theta/sigma", pview(&t_drift_sigma)},
        {"drift/theta/mu", pview(&t_drift_mu)},
        {"noise/x/sigma", pview(&x_noise_sigma)},
        {"noise/x/mu", pview(&x_noise_mu)},
        {"noise/y/sigma", pview(&y_noise_sigma)},
        {"noise/y/mu", pview(&y_noise_mu)},
        {"noise/theta/sigma", pview(&t_noise_sigma)},
        {"noise/theta/mu", pview(&t_noise_mu)}
    };
#undef pview

    //! Model for the body of the gps
    Model* sensor_model = nullptr;

    //! Body of the main part of the lidar
    b2Body* sensorBody = nullptr;

    //! Fixture of the main part of the lidar
    b2Fixture* sensorFix = nullptr;

    //! Joint connecting the main lidar body to the anchor body
    b2Joint* weldJoint = nullptr;

    //! LaserScan message being published
    std::shared_ptr<geometry_msgs::msg::Pose2D> data;

    //! Time counter
    double _timeSincePublish = 0;

    //! Box2D world in use
    b2World* _world = nullptr;

    //! Accumulated drift in x direction
    double _drift_x = 0;

    //! Accumulated drift in y direction
    double _drift_y = 0;

    //! Accumulated drift in theta
    double _drift_t = 0;

    /*!
     * \brief Augments a value using the noise, drift, and likelihood properties this object can apply.
     * Takes a ground-truth value and
     * * Decides if the output should be nan or not
     * * Adds to drift value and then applies accumulated drift
     * * Adds noise
     *
     * Drift is always calculated, regardless of if the output is nan
     *
     * \param[in] actual Ground truth value
     * \param[in, out] accumulatedDrift Total accumulated drift
     * \param[in] drift Normal Filter for generating drift
     * \param[in] drift_scale Scaling factor for drift over time
     * \param[in] noise Normal Filter for generating noise
     * \return The value resulting with noise and drift taken into account
     */
    double observe(const double& actual, double &accumulatedDrift, const double &drift_scale,
                   const NormalFilter<reng_type> &drift, const NormalFilter<reng_type> &noise);

    //! IID of plugin that generated this object
    const QString _pluginIID;

public:
    /*!
     * \brief Construct a new gps component
     * \param[in] parent QObject parent
     */
    GPS_Sensor(const QString& pluginIID, QObject* parent=nullptr);
    ~GPS_Sensor() override {}

    /*!
     * \brief Creates a new gps Component
     * \param[in] newParent QObject parent of copy
     * \return A newly constructed gps component
     */
    WorldObjectComponent *_clone(QObject *newParent) override;

    /*!
     * \brief Getter for the gps-specific properties
     * \return Mapping of gps properties by identifiers
     */
    QMap<QString, QSharedPointer<PropertyView>> _getProperties() override{
        return _properties;
    }

    /*!
     * \brief Check if component uses ROS channels
     * \return true
     */
    bool usesChannels() override{
        return true;
    }

    /*!
     * \brief Loads the component into the box2d world
     * \param[in] world Box2D world to add lidar sensor to
     * \param[in] oId object_id of the object the lidar is part of
     * \param[in] anchor Anchor body to joint self to
     */
    void _generateBodies(b2World *world, object_id oId, b2Body *anchor) override;

    //! Clears all bodies the lidar has in the Box2d world
    void _clearBodies() override;

    /*!
     * \brief Sets the ROS node to publish messages with
     * \param[in] node The ROS node to use
     */
    void _setROSNode(std::shared_ptr<rclcpp::Node> node) override;

    /*!
     * \brief Returns the plugin IID
     * \return The IID of the plugin providing this component
     */
    QString getPluginName() override { return _pluginIID; }

private slots:
    //! Refreshes ROS channel
    void _channelChanged();

    //! Creates the box2d fixture for the lidar
    void _attachSensorFixture();

    //! Builds the shapes drawn to represent the lidar
    void _buildModels();

public slots:
    //! Connects to all ROS topics
    virtual void _connectChannels() override;

    //! Disconnects all ROS topics
    virtual void _disconnectChannels() override;

    //! Updates the time since last message, and if it is time to publish a new message, accumulate drift, augment values, and publish
    virtual void _worldTicked(const double dt) override;
};
