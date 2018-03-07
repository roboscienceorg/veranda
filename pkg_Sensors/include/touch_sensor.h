#ifndef TOUCH_SENSOR_RING_H
#define TOUCH_SENSOR_RING_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include <sdsmt_simulator/world_object_component.h>
#include <Box2D/Box2D.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

class Touch_Sensor : public WorldObjectComponent
{
    Q_OBJECT

    object_id objectId = 0;
    QString _outputChannel;
    bool _connected = false;

    std::shared_ptr<rclcpp::Node> _rosNode;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr _sendChannel;

    Property output_channel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Output channel for touch messages"), "");

    Property angle_start = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Start angle of the sensors(degrees)"), QVariant(0.0),
                                    &Property::angle_validator);

    Property angle_end = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "End angle of the sensors (degrees)"), QVariant(0.0),
                                  &Property::angle_validator);

    Property radius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                               "Radius of the touch sensor ring"), QVariant(1.0),
                               &Property::abs_double_validator);

    Property sensor_count = Property(PropertyInfo(false, false, PropertyInfo::INT,
                                                  "Number of sensors on the ring"), QVariant(1),
                                                  [](QVariant _old, QVariant _new)
                                                  {
                                                        bool valid;
                                                        int newVal = _new.toInt(&valid);
                                                        if(valid && newVal >= 1)
                                                            return _new;
                                                        return _old;
                                                  });

#define pview(a) QSharedPointer<PropertyView>(new PropertyView(a))
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"channels/output_touches", pview(&output_channel)},
        {"angle_start", pview(&angle_start)},
        {"angle_end", pview(&angle_end)},
        {"ring_radius", pview(&radius)},
        {"sensor_count", pview(&sensor_count)}
    };
#undef pview

    Model* buttons_model = nullptr;
    Model* touches_model = nullptr;

    b2Body* sensorBody = nullptr;
    b2Fixture* sensorFix = nullptr;
    b2Joint* weldJoint = nullptr;

    //Data published
    std::shared_ptr<std_msgs::msg::ByteMultiArray> data;

    //Track of what's shown
    QSet<int> active_touches;

    //All possible touches
    QVector<b2Shape*> touch_image;

    b2World* _world = nullptr;

public:
    Touch_Sensor(QObject* parent=nullptr);

    WorldObjectComponent* clone(QObject *newParent);

    QMap<QString, QSharedPointer<PropertyView>> _getProperties(){
        return _properties;
    }

    bool usesChannels(){
        return true;
    }

    void generateBodies(b2World *world, object_id oId, b2Body *anchor);
    void clearBodies();

    void setROSNode(std::shared_ptr<rclcpp::Node> node);

private slots:
    void _channelChanged(QVariant);
    void _attachSensorFixture();
    void _buildModels();
    void _evaluateContact(b2Contact*, QVector<int> &newTouches, QSet<int> &touchesNow);
    void _updateDataMessageDimensions();

public slots:
    //Connects to all ROS topics
    virtual void connectChannels();

    //Disconnects all ROS topics
    virtual void disconnectChannels();

    virtual void _worldTicked(const double);
};

#endif
