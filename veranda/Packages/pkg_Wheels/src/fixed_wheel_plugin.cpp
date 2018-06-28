#include "fixed_wheel_plugin.h"
#include "fixed_wheel.h"

WorldObjectComponent* Fixed_Wheel_Plugin::createComponent()
{
    return new Fixed_Wheel;
}
