#ifndef DESIGNER_WIDGET_H
#define DESIGNER_WIDGET_H

#include <QWidget>
#include <QListWidgetItem>

#include "interfaces/simulator_ui_if.h"
#include "interfaces/simulator_visual_if.h"
#include "interfaces/world_object_wrappers.h"

#include <sdsmt_simulator/world_object_component_plugin.h>

class Designer_Widget : public QListWidgetItem
{
    typedef std::function<Simulator_Visual_If*()> visualizerFactory;

    Simulator_Visual_If* view;
    WorldObjectProperties* properties;
    //QLabel name; in properties? set tooltip
    visualizerFactory makeWidget;

public:
    Designer_Widget(WorldObjectProperties *object, visualizerFactory factory, QListWidget *parent=nullptr);
    QString getType();

private:

    
    //there will be x amount of these widgets, each one for a different TAB of either the Designer or Simulator visual widget lists   
};

#endif // DESIGNER_WIDGET_H
