#include "ackermann_steer_plugin.h"
#include "ackermann_steer.h"

Ackermann_Steer_Plugin::Ackermann_Steer_Plugin()
{

}

WorldObjectComponent_If* Ackermann_Steer_Plugin::createComponent()
{
    return new Ackermann_Steer;
}
