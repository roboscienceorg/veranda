#include "lidar_sensor_plugin.h"
#include "veranda_sensors/lidar_sensor.h"

WorldObjectComponent *Lidar_Sensor_Plugin::createComponent()
{
    return new Lidar_Sensor(LIDAR_IID);
}
