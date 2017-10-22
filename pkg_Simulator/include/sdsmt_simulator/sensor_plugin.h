#ifndef SENSOR_PLUGIN_H
#define SENSOR_PLUGIN_H

#include "sensor_if.h"

class Sensor_Plugin_If
{
public:
    virtual ~Sensor_Plugin_If(){}

    virtual Sensor_If* createSensor() = 0;
};

Q_DECLARE_INTERFACE(Sensor_Plugin_If, "org.sdsmt.2dSim.sensor")

#endif // SENSOR_PLUGIN_H
