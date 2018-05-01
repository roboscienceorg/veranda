#include "circle_plugin.h"
#include "circle.h"

WorldObjectComponent* Circle_Plugin::createComponent()
{
    return new Circle;
}
