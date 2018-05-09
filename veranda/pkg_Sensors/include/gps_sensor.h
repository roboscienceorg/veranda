//! \file
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include <veranda/world_object_component.h>
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

#include "defines.h"

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

#define pview(a) QSharedPointer<PropertyView>(new PropertyView(a))
    //! Mapping of lidar propertys by their identifiers
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
        {"noise/theta/mu", pview(&t_noise_mu)},
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
    double _drift_x;

    //! Accumulated drift in y direction
    double _drift_y;

    //! Accumulated drift in theta
    double _drift_t;

    //! Random number engine
    std::mt19937_64 _reng;

    //! Distribution for uniform probabilities
    std::uniform_real_distribution<> _uniDist = std::uniform_real_distribution<>(0.0, 1.0);

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
     * \param[in] chance Probability of the value not becoming nan
     * \param[in, out] drift Total accumulated drift
     * \param[in] drift_sigma Sigma for normal distribution of drift
     * \param[in] drift_mu Mu for normal distribution of drift
     * \param[in] drift_scale Scaling factor for drift added
     * \param[in] noise_sigma Sigma for normal distribution of noise
     * \param[in] noise_mu Mu for normal distribution of noise
     * \return The value resulting with noise and drift taken into account
     */
    double observe(const double& actual, const double& chance,
                   double &drift, const double& drift_sigma, const double& drift_mu, const double& drift_scale,
                   const double& noise_sigma, const double& noise_mu);

public:
    /*!
     * \brief Construct a new gps component
     * \param[in] parent QObject parent
     */
    GPS_Sensor(QObject* parent=nullptr);

    /*!
     * \brief Creates a new gps Component
     * \param[in] newParent QObject parent of copy
     * \return A newly constructed gps component
     */
    WorldObjectComponent *_clone(QObject *newParent);

    /*!
     * \brief Getter for the gps-specific properties
     * \return Mapping of gps properties by identifiers
     */
    QMap<QString, QSharedPointer<PropertyView>> _getProperties(){
        return _properties;
    }

    /*!
     * \brief Check if component uses ROS channels
     * \return true
     */
    bool usesChannels(){
        return true;
    }

    /*!
     * \brief Loads the component into the box2d world
     * \param[in] world Box2D world to add lidar sensor to
     * \param[in] oId object_id of the object the lidar is part of
     * \param[in] anchor Anchor body to joint self to
     */
    void generateBodies(b2World *world, object_id oId, b2Body *anchor);

    //! Clears all bodies the lidar has in the Box2d world
    void clearBodies();

    /*!
     * \brief Sets the ROS node to publish messages with
     * \param[in] node The ROS node to use
     */
    void setROSNode(std::shared_ptr<rclcpp::Node> node);

    /*!
     * \brief Getter for the name of the plugin which provides this component
     * \return "org.roboscience.veranda.worldObjectComponent.defaults.gps"
     */
    QString getPluginName() { return GPS_IID; }

private slots:
    //! Refreshes ROS channel
    void _channelChanged();

    //! Creates the box2d fixture for the lidar
    void _attachSensorFixture();

    //! Builds the shapes drawn to represent the lidar
    void _buildModels();

public slots:
    //! Connects to all ROS topics
    virtual void connectChannels();

    //! Disconnects all ROS topics
    virtual void disconnectChannels();

    //! Updates the time since last message, and if it is time to publish a new message, accumulate drift, augment values, and publish
    virtual void _worldTicked(const double dt);
};
