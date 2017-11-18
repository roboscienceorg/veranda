#ifndef FLOATER_DRIVETRAIN_H
#define FLOATER_DRIVETRAIN_H

#include "ros/ros.h"
#include "std_msgs/ByteMultiArray.h"

#include <sdsmt_simulator/sensor_if.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

class Touch_Sensor : public Sensor_If
{
    Q_OBJECT

    constexpr static double PI = 3.14159265359;
    constexpr static double RAD2DEG = 360.0/(2*PI);
    constexpr static double DEG2RAD = 1.0/RAD2DEG;

    QString _outputChannel;
    bool _connected = false;

    ros::NodeHandle _rosNode;
    ros::Publisher _sendChannel;

    static QVariant _validate_angle(QVariant _old, QVariant _new);

    Property output_channel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Output channel for touch messages"), "");

    Property angle_start = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Start angle of the sensors(degrees)"), QVariant(0.0),
                                    &_validate_angle);

    Property angle_end = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "End angle of the sensors (degrees)"), QVariant(0.0),
                                  &_validate_angle);

    Property radius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                               "Radius of the touch sensor ring"), QVariant(1.0),
                               [](QVariant _old, QVariant _new)
                               {
                                     bool valid;
                                     int newVal = _new.toDouble(&valid);
                                     if(valid && newVal >= 0)
                                         return _new;
                                     return _old;
                               });

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

    QMap<QString, PropertyView> _properties{
        {"channels/output_touches", &output_channel},
        {"angle_start", &angle_start},
        {"angle_end", &angle_end},
        {"ring_radius", &radius},
        {"sensor_count", &sensor_count}
    };

    Model* buttons_model = nullptr;
    Model* touches_model = nullptr;

    b2Body* sensorBody = nullptr;
    b2Fixture* sensorFix = nullptr;

    //Data published
    std_msgs::ByteMultiArray data;

    //Track of what's shown
    QSet<int> active_touches;

    //All possible touches
    QVector<b2Shape*> touch_image;

public:
    Touch_Sensor(QObject* parent=nullptr);

    depracatedWorldObject_If* clone(QObject *newParent);

    virtual QMap<QString, PropertyView>& getAllProperties(){
        return _properties;
    }

    virtual QString propertyGroupName(){ return "Touch Ring"; }

    QVector<Model*> getModels(){ return {buttons_model, touches_model}; }

    uint64_t dynamicBodiesRequired(){ return 1; }
    QVector<b2JointDef*> setDynamicBodies(QVector<b2Body *> & bodies);
    void clearDynamicBodies();

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

    virtual void worldTicked(const b2World*, const double&);
};

#endif // FLOATER_DRIVETRAIN_H
