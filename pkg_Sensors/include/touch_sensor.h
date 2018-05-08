//! \file
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include <sdsmt_simulator/world_object_component.h>
#include <Box2D/Box2D.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

#include "defines.h"

/*!
 * \brief Component which generates touch event messages when contacted
 * The Touch Ring sensor component simulates having a ring of bump sensors.
 * The ring can be any radius, and have any number of buttons on it. The area
 * sensed can be restricted to a slice of the circle. Buttons will be evenly
 * spaced through the area sensed.
 *
 * The component publishes the byte_multi_array type any time the state of
 * one of the buttons changes. Each bytes will be either 0 or non-0, representing
 * not pressed and pressed, respectively. Each element i in the message is
 * mapped to the button i on the touch ring, traveling counter clockwise from
 * the start of the ring
 * \todo Check which side of the ring is the start and note that here
 */
class Touch_Sensor : public WorldObjectComponent
{
    Q_OBJECT

    //! Id of the WorldObject the sensor is part of
    object_id objectId = 0;

    //! ROS channel to publish on
    QString _outputChannel;

    //! Flag indicating if the ROS channel is connected
    bool _connected = false;

    //! ROS node to publish messages with
    std::shared_ptr<rclcpp::Node> _rosNode;

    //! ROS channel to publish ByteMultiArray messages on
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr _sendChannel;

    //! Property: ROS channel to publish on
    Property output_channel = Property(PropertyInfo(false, true, false, PropertyInfo::STRING,
                                                    "Output channel for touch messages"), "");

    //! Property: Starting angle of the sensed area
    Property angle_start = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE,
                                    "Start angle of the sensors(degrees)"), QVariant(0.0),
                                    &Property::angle_validator);

    //! Property: Ending angle of the sensed area (Counterclockwise from start)
    Property angle_end = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE,
                                  "End angle of the sensors (degrees)"), QVariant(360.0),
                                  &Property::angle_validator);

    //! Property: Radius of the bump ring (meters)
    Property radius = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE,
                               "Radius of the touch sensor ring"), QVariant(1.0),
                               &Property::abs_double_validator);

    //! Property: Number of distinct bump locations
    Property sensor_count = Property(PropertyInfo(false, true, false, PropertyInfo::INT,
                                                  "Number of sensors on the ring"), QVariant(10),
                                                  [](QVariant _old, QVariant _new)
                                                  {
                                                        bool valid;
                                                        int newVal = _new.toInt(&valid);
                                                        if(valid && newVal >= 1)
                                                            return _new;
                                                        return _old;
                                                  });

#define pview(a) QSharedPointer<PropertyView>(new PropertyView(a))
    //! Property: Rate of publishing messages
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"channels/output_touches", pview(&output_channel)},
        {"angle_start", pview(&angle_start)},
        {"angle_end", pview(&angle_end)},
        {"ring_radius", pview(&radius)},
        {"sensor_count", pview(&sensor_count)}
    };
#undef pview

    //! Model for the body of the ring
    Model* buttons_model = nullptr;

    //! Model for the bump indicators
    Model* touches_model = nullptr;

    //! Body of the main part of the ring
    b2Body* sensorBody = nullptr;

    //! Fixture of the main part of the ring
    b2Fixture* sensorFix = nullptr;

    //! Joint connecting the main ring body to the anchor body
    b2Joint* weldJoint = nullptr;

    //! ByteMultiArray message being published
    std::shared_ptr<std_msgs::msg::ByteMultiArray> data;

    //! Set containing the indices of all the buttons currently known to be pressed
    QSet<int> active_touches;

    //! Group of all the images to draw to represent button touches
    QVector<b2Shape*> touch_image;

    //! Box2D world in use
    b2World* _world = nullptr;

public:
    /*!
     * \brief Construct a new touch ring component
     * \param[in] parent QObject parent
     */
    Touch_Sensor(QObject* parent=nullptr);

    /*!
     * \brief Creates a new Touch Ring Component
     * \param[in] newParent QObject parent of copy
     * \return A newly constructed Touch Ring component
     */
    WorldObjectComponent* _clone(QObject *newParent);

    /*!
     * \brief Getter for the touch ring-specific properties
     * \return Mapping of touch ring properties by identifiers
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
     * \param[in] world Box2D world to add touch ring sensor to
     * \param[in] oId object_id of the object the lidar is part of
     * \param[in] anchor Anchor body to joint self to
     */
    void generateBodies(b2World *world, object_id oId, b2Body *anchor);

    //! Clears all bodies the ring has in the Box2d world
    void clearBodies();

    /*!
     * \brief Sets the ROS node to publish messages with
     * \param[in] node The ROS node to use
     */
    void setROSNode(std::shared_ptr<rclcpp::Node> node);

    /*!
     * \brief Getter for the name of the plugin which provides this component
     * \return "org.sdsmt.sim.2d.worldObjectComponent.defaults.touchring"
     */
    QString getPluginName() { return TOUCH_IID; }

private slots:
    //! Refreshes ROS channel
    void _channelChanged();

    //! Creates the box2d fixture for the ring
    void _attachSensorFixture();

    //! Builds the shapes drawn to represent the ring and touches
    void _buildModels();

    /*!
     * \brief Checks a contact in the world to see if it adds new button bumps
     * \param[in] c Contact to evaluate
     * \param[out] newTouches Set of previously unknown touches indicated by this contact
     * \param[in,out] touchesNow
     */
    void _evaluateContact(b2Contact* c, QVector<int> &newTouches, QSet<int> &touchesNow);

    //! Resizes the message
    void _updateDataMessageDimensions();

public slots:
    //! Connects to all ROS topics
    virtual void connectChannels();

    //! Disconnects all ROS topics
    virtual void disconnectChannels();

    //! Checks for button states to change, and if so publishes a message
    virtual void _worldTicked(const double);
};
