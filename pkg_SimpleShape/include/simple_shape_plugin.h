#ifndef SIMPLE_SHAPE_PLUGIN_H
#define SIMPLE_SHAPE_PLUGIN_H

#include <QObject>

#include <sdsmt_simulator/world_object_component_plugin.h>

class Simple_Shape_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.sdsmt.sim.2d.worldObjectComponent.defaults.simpleshape")
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    Simple_Shape_Plugin();
    WorldObjectComponent_If* createComponent();

};

#endif
