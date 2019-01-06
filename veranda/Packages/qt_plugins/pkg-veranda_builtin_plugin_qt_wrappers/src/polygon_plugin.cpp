#include "polygon_plugin.h"
#include "veranda_shapes/polygon.h"

WorldObjectComponent* Polygon_Plugin::createComponent()
{
    return new Polygon(POLYGON_IID);
}
