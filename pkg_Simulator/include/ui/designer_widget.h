#ifndef DESIGNER_WIDGET_H
#define DESIGNER_WIDGET_H

#include <QWidget>
#include <QListWidgetItem>

#include "interfaces/simulator_visual_if.h"
#include "interfaces/world_object_wrappers.h"

#include <sdsmt_simulator/world_object_component_plugin.h>
#include <sdsmt_simulator/world_object_loader_if.h>
#include <sdsmt_simulator/world_object_saver_if.h>

class Designer_Widget : public QListWidgetItem
{
    Q_OBJECT

public:
    Designer_Widget(WorldObjectProperties *object);
    QString getType();

private:
    Simulator_Visual_If* view;
    WorldObjectProperties* properties;
    //QLabel name; in properties? set tooltip
    visualizerFactory makeWidget;
    
    //there will be x amount of these widgets, each one for a different TAB of either the Designer or Simulator visual widget lists   
};

#endif // DESIGNER_WIDGET_H
