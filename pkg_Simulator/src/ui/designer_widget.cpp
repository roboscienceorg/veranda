#include "include/ui/designer_widget.h"

Designer_Widget::Designer_Widget(WorldObjectComponent *object, WorldObjectProperties *object2, visualizerFactory factory, QListWidget *parent) :
              QListWidgetItem(parent)
{
    view = factory();
    component = object;
    properties = object2;

    QHBoxLayout* tileLayout = new QHBoxLayout();
    tileLayout->addWidget(view);

    //set tooltip to be property info for key "name"
    setToolTip(properties->getName());

    view->objectAddedToScreen(properties->getModels(), 0);

    QPixmap pixmap(view->size());
    view->render(&pixmap);
    setIcon(QIcon(pixmap));

}

