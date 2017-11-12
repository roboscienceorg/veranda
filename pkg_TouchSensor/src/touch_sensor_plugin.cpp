#include "touch_sensor_plugin.h"
#include "touch_sensor.h"

Touch_Sensor_Plugin::Touch_Sensor_Plugin()
{

}

Sensor_If* Touch_Sensor_Plugin::createSensor()
{
    return new Touch_Sensor;
}
