#include "simple_shape_plugin.h"
#include "simple_shape.h"

Simple_Shape_Plugin::Simple_Shape_Plugin()
{

}

WorldObjectComponent_If* Simple_Shape_Plugin::createComponent()
{
    return new Simple_Shape;
}
