#include "fixed_wheel_plugin.h"
#include "fixed_wheel.h"

Fixed_Wheel_Plugin::Fixed_Wheel_Plugin()
{

}

WorldObjectComponent* Fixed_Wheel_Plugin::createComponent()
{
    return new Fixed_Wheel;
}
