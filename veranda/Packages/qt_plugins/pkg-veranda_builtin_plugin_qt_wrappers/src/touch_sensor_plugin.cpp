#include "touch_sensor_plugin.h"
#include "veranda_sensors/touch_sensor.h"

WorldObjectComponent *Touch_Sensor_Plugin::createComponent()
{
    return new Touch_Sensor(TOUCH_IID);
}
