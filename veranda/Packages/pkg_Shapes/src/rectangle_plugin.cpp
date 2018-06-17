#include "rectangle_plugin.h"
#include "rectangle.h"

WorldObjectComponent* Rectangle_Plugin::createComponent()
{
    return new Rectangle;
}
