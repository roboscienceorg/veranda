#include "include/ui/designer_widget.h"

Designer_Widget::Designer_Widget(WorldObjectComponent *object, WorldObjectProperties* object2, visualizerFactory factory, QListWidget *parent) :
              QListWidgetItem(parent)
{
    view = factory();
    component = object;
    properties = object2;

    //set tooltip to be property info for key "name"
    setToolTip(properties->getName());

    view->setWorldBounds(-1, 1, -1, 1);
    view->objectAddedToScreen(properties->getModels(), 0);

    QPixmap pixmap(view->size());
    view->render(&pixmap);
    setIcon(QIcon(pixmap));

}
