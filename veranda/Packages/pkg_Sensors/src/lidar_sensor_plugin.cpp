#include "lidar_sensor_plugin.h"
#include "lidar_sensor.h"

WorldObjectComponent *Lidar_Sensor_Plugin::createComponent()
{
    return new Lidar_Sensor;
}
