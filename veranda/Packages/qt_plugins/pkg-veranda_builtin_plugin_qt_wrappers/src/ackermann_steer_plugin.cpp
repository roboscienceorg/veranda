#include "ackermann_steer_plugin.h"
#include "veranda_wheels/ackermann_steer.h"

WorldObjectComponent* Ackermann_Steer_Plugin::createComponent()
{
    return new Ackermann_Steer(ACKERMANN_IID);
}
