#include "include/ui/designer_widget.h"

Designer_Widget::Designer_Widget(WorldObjectComponent *object, WorldObjectProperties* object2, visualizerFactory factory, QListWidget *parent) :
              QListWidgetItem(parent)
{
    view = factory();
    component = object;
    properties = object2;

    //set tooltip to be property info for key "name"
    setText(properties->getName());

    view->setWorldBounds(-1.8, 1.8, -1.2, 1.2);
    view->objectAddedToScreen(properties->getModels(), 0);

    //set pixmap to size of view minus obnoxious black border of indetermined origin
    QPixmap pixmap(view->width() - 40, view->height() - 75);
    view->render(&pixmap);
    setIcon(QIcon(pixmap));
}
