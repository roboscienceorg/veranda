#include "fixed_wheel_plugin.h"
#include "fixed_wheel.h"

Touch_Sensor_Plugin::Touch_Sensor_Plugin()
{

}

WorldObjectComponent_If* Touch_Sensor_Plugin::createComponent()
{
    return new Touch_Sensor;
}
