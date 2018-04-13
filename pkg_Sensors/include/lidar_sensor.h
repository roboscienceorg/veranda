//! \file
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <sdsmt_simulator/world_object_component.h>
#include <Box2D/Box2D.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>
#include <QPair>

#include <memory>
#include <limits>
#include <cmath>

#include "defines.h"

/*!
 * \brief Component which generates distance readings like a lidar would
 * The Lidar sensor component can be added to a world object in order to sense
 * the distance between the lidar and objects around it. The lidar has an angle
 * range of some theta, which is measured from -theta/2 to +theta/2, centered on the
 * front of the lidar. There must be at least 2 rays extendingout from the lidar, one
 * at the start of the range, and one at the end (if the range is 360 degress, then these
 * rays will be the same) Distances beyond the radius of the lidar will be reported
 * as inf (as defined by std::numeric_limits<float32>::infinity()). The message published
 * by the lidar component is the standard ROS laser_scan message.
 */
class Lidar_Sensor : public WorldObjectComponent
{
    Q_OBJECT

    /*!
     * \brief Callback handler for Box2d Raycasting
     * The Box2D world can perform a raycast from a point
     * in a direction and return all points hit. This type
     * is passed to the callback to handle the point hits.
     *
     * This listener can start a raycast from a point in the
     * direction of another point and record the closest point
     * to the source. The result of a raycast is a pair containing
     * the closest point to the source, and the distance between the
     * source and that point.
     *
     * Ray casts take into account the WorldObject ids in order
     * to prevent the lidar sensing the object that it is a part of. The
     * raycast also ignores any box2d fixture which is flagged as being a
     * sensor (in terms of box2d)
     */
    class LidarRayCaster : public b2RayCastCallback
    {
        //! Record of closest point found so far
        b2Vec2 _bestPoint;

        //! Record of closest distance found so far
        double _bestDist;

        //! Record of the origin point
        b2Vec2 _startPoint;

        //! Record of the object id to ignore when raycasting
        int64_t _collisionGroup;

    public:
        /*!
         * \brief Handler callback for the raycast
         * If the raycast finds a point on a sensor (with respect to box2d) or a part
         * of the same object as the lidar, the point is ignored and the raycast continues.
         * Otherwise, the point is compared to the last closest point to see if it is closer to the
         * origin point
         * \param[in] fixture Fixture that the contact point is part of
         * \param[in] point Global position of the contact point
         * \param[in] normal See Box2d documentation of raycasting; this parameter is unused here
         * \param[in] fraction Percentage of distance along the ray to the contact point
         * \return 1 if the point is ignored, the value passed as the fraction otherwise
         */
        float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
        {
            //Ignore non-colliding fixtures
            if(fixture->IsSensor() || fixture->GetFilterData().groupIndex == _collisionGroup) return 1;

            double d = b2DistanceSquared(_startPoint, point);
            if(d < _bestDist)
            {
                _bestDist = d;
                _bestPoint = point;
            }

            return fraction;
        }

        /*!
         * \brief Resets the member values of the raycaster object and performs a new raycast
         * \param[in] world Box2d World to raycast in
         * \param[in] p1 Origin of the raycast
         * \param[in] p2 Point to raycast towards from p1
         * \param[in] collisonGroup Box2d filter group index to ignore hits in
         * \return Pair - The closest point hit by the raycast, and the distance to it from the cast origin. If no hits are found, this is p2 and infinity
         */
        QPair<b2Vec2, double> rayCast(const b2World* world, b2Vec2 p1, b2Vec2 p2, int64_t collisonGroup)
        {
            _bestDist = std::numeric_limits<double>::infinity();
            _bestPoint = p2;
            _startPoint = p1;
            _collisionGroup = collisonGroup;

            world->RayCast(this, p1, p2);

            return {_bestPoint, std::sqrt(_bestDist)};
        }
    };

    //! Size of the lidar body when drawn on screen
    constexpr static double SHAPE_RADIUS = 0.5;

    //! Id of the WorldObject the lidar is part of
    object_id objectId = 0;

    //! ROS channel to publish on
    QString _outputChannel;

    //! Flag indicating if the ROS channel is connected
    bool _connected = false;

    //! ROS node to publish messages with
    std::shared_ptr<rclcpp::Node> _rosNode;

    //! ROS channel to publish LaserScan messages on
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _sendChannel;

    //! Property: ROS channel to publish on
    Property output_channel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Output channel for touch messages"), "");

    //! Property: Total range (degrees) covered by the lidar
    Property angle_range = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Total angle covered by the scan (degrees)"), QVariant(360),
                                    &Property::angle_validator);

    //! Property: Length of lidar rays (meters)
    Property radius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "Radius of the scan (meters)"), QVariant(1),
                                  &Property::angle_validator);

    //! Property: Number of lidar rays
    Property scan_points = Property(PropertyInfo(false, false, PropertyInfo::INT,
                                                  "Number of points per scan"), QVariant(10),
                                                  [](QVariant _old, QVariant _new)
                                                  {
                                                        bool valid;
                                                        int newVal = _new.toInt(&valid);
                                                        if(valid && newVal >= 2)
                                                            return _new;
                                                        return _old;
                                                  });

    //! Property: Rate of publishing messages
    Property pub_rate = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE, "Scan rate (hz)"),
                                 QVariant(10), &Property::abs_double_validator);

#define pview(a) QSharedPointer<PropertyView>(new PropertyView(a))
    //! Mapping of lidar propertys by their identifiers
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"channels/output_ranges", pview(&output_channel)},
        {"scan_range", pview(&angle_range)},
        {"scan_radius", pview(&radius)},
        {"scan_points", pview(&scan_points)},
        {"scan_rate", pview(&pub_rate)}
    };
#undef pview

    //! Model for the body of the lidar
    Model* sensor_model = nullptr;

    //! Model for the raycast lines
    Model* scan_model = nullptr;

    //! Body of the main part of the lidar
    b2Body* sensorBody = nullptr;

    //! Fixture of the main part of the lidar
    b2Fixture* sensorFix = nullptr;

    //! Joint connecting the main lidar body to the anchor body
    b2Joint* weldJoint = nullptr;

    //! LaserScan message being published
    std::shared_ptr<sensor_msgs::msg::LaserScan> data;

    //! Vector of all ray cast lines that exist
    QVector<b2Shape*> scan_image;

    //! Raycast callback object
    LidarRayCaster _rayCaster;

    //! Time counter
    double _timeSinceScan = 0;

    //! Box2D world in use
    b2World* _world = nullptr;

public:
    /*!
     * \brief Construct a new lidar component
     * \param[in] parent QObject parent
     */
    Lidar_Sensor(QObject* parent=nullptr);

    /*!
     * \brief Creates a new Lidar Component
     * \param[in] newParent QObject parent of copy
     * \return A newly constructed Lidar component
     */
    WorldObjectComponent *_clone(QObject *newParent);

    /*!
     * \brief Getter for the lidar-specific properties
     * \return Mapping of lidar properties by identifiers
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
     * \return "org.sdsmt.sim.2d.worldObjectComponent.defaults.lidar"
     */
    QString getPluginName() { return LIDAR_IID; }

private:
    /*!
     * \brief Gets the point a specific distance (0, 0) at a specific angle relative
     * \param[in] angle_rad Angle to get point at (radians)
     * \param[in] dist Distance away to get point (meters)
     * \return The point dist meters away at the specified angle, relative to the origin (0, 0)
     */
    b2Vec2 _getRayPoint(double angle_rad, double dist);

private slots:
    //! Refreshes ROS channel
    void _channelChanged();

    //! Creates the box2d fixture for the lidar
    void _attachSensorFixture();

    //! Builds the shapes drawn to represent the lidar
    void _buildModels();

    //! Resizes the array portions of the LaserScan message
    void _updateDataMessageDimensions();

public slots:
    //! Connects to all ROS topics
    virtual void connectChannels();

    //! Disconnects all ROS topics
    virtual void disconnectChannels();

    //! Updates the time since last message, and if it is time to publish a new message, the lidar ranges are recomputed
    virtual void _worldTicked(const double dt);
};
