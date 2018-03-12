#include "include/ui/designer_widget.h"

Designer_Widget::Designer_Widget(WorldObjectProperties *object, visualizerFactory factory, QMap<QString, WorldObjectComponent_Plugin_If *> components,
                                 QVector<WorldObjectLoader_If*> loaders, QVector<WorldObjectSaver_If*> savers, QWidget *parent) :
              QListWidgetItem(this),
              Simulator_Ui_If(parent),
              makeWidget(factory), componentPlugins(components)
{
    view = makeWidget();
    properties = object;

    //set tooltip to be property info for key "name"
    QMap<QString, PropertyView>& ppts = properties->getProperties();
    QStandardItemModel* model = propertiesModel;

    for(int i=0; i<ppts.size(); i++)
    {
        QString key = model->data(model->index(i, 0)).toString();
        if(key == "name")
            setToolTip(model->index(i, 1), ppts[key].get().toString());

    }
}

QString Designer_Widget::getType()
{
    //return type as a string
    QMap<QString, PropertyView>& ppts = properties->getProperties();
    QStandardItemModel* model = propertiesModel;

    for(int i=0; i<ppts.size(); i++)
    {
        QString key = model->data(model->index(i, 0)).toString();
        if(key == "type")
            return model->index(i, 1), ppts[key].get().toString();
    }
}
