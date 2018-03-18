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

public:
    Simulator_Visual_If* view;
    WorldObjectComponent* component;
    WorldObjectProperties* properties;

    Designer_Widget(WorldObjectComponent *object, WorldObjectProperties* object2, visualizerFactory factory, QListWidget *parent=nullptr);

private:
    
    //there will be x amount of these widgets, each one for a different TAB of the Designer visual widget lists
};

#endif // DESIGNER_WIDGET_H
