#include "lidar_sensor_plugin.h"
#include "lidar_sensor.h"

Touch_Sensor_Plugin::Touch_Sensor_Plugin()
{

}

WorldObjectComponent_If* Touch_Sensor_Plugin::createComponent()
{
    return new Lidar_Sensor;
}
