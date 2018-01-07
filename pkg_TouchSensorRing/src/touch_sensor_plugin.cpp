#include "touch_sensor_plugin.h"
#include "touch_sensor.h"

Touch_Sensor_Plugin::Touch_Sensor_Plugin()
{

}

WorldObjectComponent_If* Touch_Sensor_Plugin::createComponent()
{
    return new Touch_Sensor;
}
