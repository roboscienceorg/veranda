#include "circle_plugin.h"
#include "circle.h"

Circle_Plugin::Circle_Plugin()
{

}

WorldObjectComponent* Circle_Plugin::createComponent()
{
    return new Circle;
}
