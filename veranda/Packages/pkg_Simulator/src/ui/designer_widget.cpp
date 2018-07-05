#include "include/ui/designer_widget.h"

Designer_Widget::Designer_Widget(WorldObjectComponent *object, WorldObjectProperties* object2, visualizerFactory factory, QListWidget *parent, bool simulator) :
              QListWidgetItem(parent)
{
    view = factory();
    component = object;
    properties = object2;

    //set tooltip to be property info for key "name"
    setText(properties->getName());

    view->setNavigationEnabled(false);
    view->objectAddedToScreen(properties->getModels(), 1);
    view->nothingSelected();

    //set pixmap to size of view minus obnoxious black border of indetermined origin
    QPixmap pixmap(view->width()/* - 40*/, view->height()/* - 75*/);
    view->render(&pixmap);
    setIcon(QIcon(pixmap));
}
