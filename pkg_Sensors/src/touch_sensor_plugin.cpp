#include "touch_sensor_plugin.h"
#include "touch_sensor.h"

WorldObjectComponent *Touch_Sensor_Plugin::createComponent()
{
    return new Touch_Sensor;
}
