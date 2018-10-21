#include "swedish_wheel_plugin.h"
#include "swedish_wheel.h"

WorldObjectComponent* Swedish_Wheel_Plugin::createComponent()
{
    return new Swedish_Wheel;
}
