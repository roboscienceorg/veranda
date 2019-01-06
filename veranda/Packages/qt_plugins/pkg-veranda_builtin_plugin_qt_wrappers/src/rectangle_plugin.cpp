#include "rectangle_plugin.h"
#include "veranda_shapes/rectangle.h"

WorldObjectComponent* Rectangle_Plugin::createComponent()
{
    return new Rectangle(RECT_IID);
}
