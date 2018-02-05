#include "rectangle_plugin.h"
#include "rectangle.h"

Rectangle_Plugin::Rectangle_Plugin()
{

}

WorldObjectComponent_If* Rectangle_Plugin::createComponent()
{
    return new Rectangle;
}
