#ifndef TOUCH_SENSOR_PLUGIN_H
#define TOUCH_SENSOR_PLUGIN_H

#include <QObject>

#include <sdsmt_simulator/sensor_plugin.h>

class Touch_Sensor_Plugin : public QObject, public Sensor_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.sdsmt.2dSim.sensor.touchring")
    Q_INTERFACES(Sensor_Plugin_If)

public:
    Touch_Sensor_Plugin();
    Sensor_If* createSensor();

};

#endif
