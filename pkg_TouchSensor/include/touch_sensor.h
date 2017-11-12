#ifndef FLOATER_DRIVETRAIN_H
#define FLOATER_DRIVETRAIN_H

#include "ros/ros.h"
#include "std_msgs/ByteMultiArray.h"

#include <sdsmt_simulator/sensor_if.h>

#include <QVector>
#include <QString>
#include <QObject>

class Touch_Sensor : public Sensor_If
{
    Q_OBJECT

    QString _outputChannel;
    bool _connected = false;

    ros::NodeHandle _rosNode;
    ros::Publisher _sendChannel;

    Property output_channel = Property(PropertyInfo(false, false, PropertyInfo::STRING,
                                                    "Output channel for touch messages"), "");

    Property angle_start = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                    "Start angle of the sensors(degrees)"), QVariant(0),
                                    &Property::double_validator);

    Property angle_end = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                                  "End angle of the sensors (degrees)"), QVariant(0),
                                  &Property::double_validator);

    Property radius = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                               "Radius of the touch sensor ring"), QVariant(0),
                               &Property::double_validator);

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

    std_msgs::ByteMultiArray data;

public:
    Touch_Sensor(QObject* parent=nullptr);

    WorldObject_If* clone(QObject *newParent);

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
    void _buildButtonModel();
    void _updateTouchesModel();

public slots:
    //Connects to all ROS topics
    virtual void connectChannels();

    //Disconnects all ROS topics
    virtual void disconnectChannels();

    virtual void worldTicked(const b2World*, const double& t);
};

#endif // FLOATER_DRIVETRAIN_H
