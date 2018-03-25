#ifndef TOUCH_SENSOR_RING_H
#define TOUCH_SENSOR_RING_H

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

class Lidar_Sensor : public WorldObjectComponent
{
    Q_OBJECT

    class LidarRayCaster : public b2RayCastCallback
    {
        b2Vec2 _bestPoint;
        double _bestDist;

        b2Vec2 _startPoint;

        int64_t _collisionGroup;

    public:
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

    constexpr static double SHAPE_RADIUS = 0.5;

    object_id objectId = 0;
    QString _outputChannel;
    bool _connected = false;

    std::shared_ptr<rclcpp::Node> _rosNode;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _sendChannel;

    Property output_channel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Output channel for touch messages"), "");

    Property angle_range = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Total angle covered by the scan (degrees)"), QVariant(360),
                                    &Property::angle_validator);

    Property radius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "Radius of the scan (meters)"), QVariant(1),
                                  &Property::angle_validator);

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

    Property pub_rate = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE, "Scan rate (hz)"),
                                 QVariant(10), &Property::abs_double_validator);

#define pview(a) QSharedPointer<PropertyView>(new PropertyView(a))
    QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"channels/output_ranges", pview(&output_channel)},
        {"scan_range", pview(&angle_range)},
        {"scan_radius", pview(&radius)},
        {"scan_points", pview(&scan_points)},
        {"scan_rate", pview(&pub_rate)}
    };
#undef pview

    Model* sensor_model = nullptr;
    Model* scan_model = nullptr;

    b2Body* sensorBody = nullptr;
    b2Fixture* sensorFix = nullptr;
    b2Joint* weldJoint = nullptr;

    //Data published
    std::shared_ptr<sensor_msgs::msg::LaserScan> data;

    //All possible touches
    QVector<b2Shape*> scan_image;

    LidarRayCaster _rayCaster;

    double _timeSinceScan = 0;

    b2World* _world = nullptr;

public:
    Lidar_Sensor(QObject* parent=nullptr);

    WorldObjectComponent *_clone(QObject *newParent);

    QMap<QString, QSharedPointer<PropertyView>> _getProperties(){
        return _properties;
    }

    bool usesChannels(){
        return true;
    }

    void generateBodies(b2World *world, object_id oId, b2Body *anchor);
    void clearBodies();

    void setROSNode(std::shared_ptr<rclcpp::Node> node);

    QString getPluginName() { return LIDAR_IID; }

private:
    b2Vec2 _getRayPoint(double angle_rad, double dist);

private slots:
    void _channelChanged(QVariant);
    void _attachSensorFixture();
    void _buildModels();
    void _updateDataMessageDimensions();

public slots:
    //Connects to all ROS topics
    virtual void connectChannels();

    //Disconnects all ROS topics
    virtual void disconnectChannels();

    virtual void _worldTicked(const double dt);
};

#endif // FLOATER_DRIVETRAIN_H
