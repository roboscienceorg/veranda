#ifndef TOUCH_SENSOR_RING_H
#define TOUCH_SENSOR_RING_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <sdsmt_simulator/world_object_component_if.h>
#include <Box2D/Box2D.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>
#include <QPair>

#include <memory>
#include <limits>
#include <cmath>

class Lidar_Sensor : public WorldObjectComponent_If
{
    Q_OBJECT

    class LidarRayCaster : public b2RayCastCallback
    {
        b2Vec2 _bestPoint;
        double _bestDist;

        b2Vec2 _startPoint;

    public:
        float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
        {
            double d = b2DistanceSquared(_startPoint, point);
            if(d < _bestDist)
            {
                _bestDist = d;
                _bestPoint = point;
            }

            return fraction;
        }

        QPair<b2Vec2, double> rayCast(const b2World* world, b2Vec2 p1, b2Vec2 p2)
        {
            _bestDist = std::numeric_limits<double>::infinity();
            _bestPoint = p2;
            _startPoint = p1;

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

    Property x_local = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "X location of component within object"),
                                QVariant(0.0), &Property::double_validator);

    Property y_local = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Y location of component within object"),
                                QVariant(0.0), &Property::double_validator);

    Property theta_local = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Angle of component within object"),
                                QVariant(0.0), &Property::angle_validator);

    Property output_channel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Output channel for touch messages"), "");

    Property angle_range = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Total angle covered by the scan (degrees)"), QVariant(0.0),
                                    &Property::angle_validator);

    Property radius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "Radius of the scan (degrees)"), QVariant(0.0),
                                  &Property::angle_validator);

    Property scan_points = Property(PropertyInfo(false, false, PropertyInfo::INT,
                                                  "Number of points per scan"), QVariant(2),
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

    QMap<QString, PropertyView> _properties{
        {"channels/output_ranges", &output_channel},
        {"x_local", &x_local},
        {"y_local", &y_local},
        {"theta_local", &theta_local},
        {"scan_range", &angle_range},
        {"scan_radius", &radius},
        {"scan_points", &scan_points},
        {"scan_rate", &pub_rate}
    };

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

public:
    Lidar_Sensor(QObject* parent=nullptr);

    WorldObjectComponent_If* clone(QObject *newParent);

    QMap<QString, PropertyView>& getProperties(){
        return _properties;
    }

    QString getPropertyGroup(){
        return "Lidar";
    }

    QVector<Model*> getModels(){
        return {sensor_model, scan_model};
    }

    bool usesChannels(){
        return true;
    }

    QVector<b2Body *> generateBodies(b2World *world, object_id oId, b2Body *anchor);
    void clearBodies(b2World *world);

    void setROSNode(std::shared_ptr<rclcpp::Node> node);

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

    virtual void worldTicked(const b2World*world, const double dt);

    virtual void setObjectMass(double mass){}
};

#endif // FLOATER_DRIVETRAIN_H
