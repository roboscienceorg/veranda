#include "include/ui/designer_widget.h"

Designer_Widget::Designer_Widget(WorldObjectProperties *object, visualizerFactory factory, QListWidget *parent) :
              QListWidgetItem(parent)
{
    makeWidget = factory;

    view = makeWidget();
    properties = object;

    //set tooltip to be property info for key "name"

    setToolTip(properties->getName());
}

QString Designer_Widget::getType()
{
    setToolTip(properties->getType());
}
