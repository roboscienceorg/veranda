#include "polygon_plugin.h"
#include "polygon.h"

Polygon_Plugin::Polygon_Plugin()
{

}

WorldObjectComponent* Polygon_Plugin::createComponent()
{
    return new Polygon;
}
