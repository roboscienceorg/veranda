#include "gps_sensor_plugin.h"
#include "veranda_sensors/gps_sensor.h"

WorldObjectComponent *GPS_Sensor_Plugin::createComponent()
{
    return new GPS_Sensor(GPS_IID);
}
