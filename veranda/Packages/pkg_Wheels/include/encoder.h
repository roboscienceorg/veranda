#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <veranda/filter.h>
#include <veranda/world_object_component.h>
#include <Box2D/Box2D.h>

class Encoder : public WorldObjectComponent
{
    Q_OBJECT

    typedef std::mt19937_64 reng_type;

    //! ROS channel to publish on
    QString _outputChannel;

    //! ROS node to publish messages with
    std::shared_ptr<rclcpp::Node> _rosNode;

    //! ROS channel to publish gps messages on
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _sendChannel;

    std::shared_ptr<std_msgs::msg::Float32> _sendMessage;

    //! Property: ROS channel to publish on
    Property output_channel = Property(PropertyInfo(false, true, false, PropertyInfo::STRING,
                                                    "Output channel for encoder messages"), "");

    //! Property: Rate of publishing messages
    Property pub_rate = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Publish rate (hz)"),
                                 QVariant(10), &Property::abs_double_validator);


    //! Property: Noise is readings (sigma)
    Property noise_sigma = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in readings (sigma)"),
                                QVariant(0.0), &Property::abs_double_validator);

    //! Property: Noise in readings (mu)
    Property noise_mu = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Noise in readings (mu)"),
                                QVariant(0.0), &Property::double_validator);

    //! Filter for noise
    NormalFilter<reng_type> noise_filter;

#define pview(a) QSharedPointer<PropertyView>(new PropertyView(a))
    //! Mapping of encoder propertys by their identifiers
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"channels/angular_velocity", pview(&output_channel)},
        {"channels/angular_velocity/publish_rate", pview(&pub_rate)},
        {"noise/sigma", pview(&noise_sigma)},
        {"noise/mu", pview(&noise_mu)}
    };
#undef pview

    b2Body* _wheelBody;
    double _wheelRadius;
    double _lastPublish = 0;

    void _refreshChannel(QVariant);
    double _calculateAngularVelocity(const b2Body* body, const double& radius);

protected:
    WorldObjectComponent* _clone(QObject* parent) override;
    void setROSNode(std::shared_ptr<rclcpp::Node> node) override;
    void connectChannels() override;
    void disconnectChannels() override;
    void _worldTicked(double) override;
    QString getPluginName() override { return ""; }

public:
    Encoder(QObject* parent);
    ~Encoder() override {}

    void setWheel(b2Body* wheelBody, double radius);
};
