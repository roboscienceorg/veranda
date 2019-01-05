#include "polygon_plugin.h"
#include "polygon.h"

WorldObjectComponent* Polygon_Plugin::createComponent()
{
    return new Polygon;
}
