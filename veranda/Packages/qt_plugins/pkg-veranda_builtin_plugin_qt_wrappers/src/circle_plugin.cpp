#include "circle_plugin.h"
#include "veranda_shapes/circle.h"

WorldObjectComponent* Circle_Plugin::createComponent()
{
    return new Circle(CIRCLE_IID);
}
