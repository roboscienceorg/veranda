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

    //setBackgroundColor(Qt::white);
    //view->setMinimumSize(QSize(100, 100));
    //view->setMaximumSize(QSize(150, 150));
    //setTextAlignment(Qt::AlignLeft);
    //setStyleSheet("QListWidgetItem { background-color: white; }");

    QPixmap pixmap(view->size());
    view->render(&pixmap);
    setIcon(QIcon(pixmap));
    setSizeHint(QSize(164,164));
    //QIcon icon = new QIcon()
    //icon()->setScaledContents(true);
}
