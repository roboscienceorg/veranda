#ifndef FIXED_WHEEL_PLUGIN_H
#define FIXED_WHEEL_PLUGIN_H

#include <QObject>

#include <sdsmt_simulator/world_object_component_plugin.h>

class Fixed_Wheel_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel")
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    Fixed_Wheel_Plugin();
    WorldObjectComponent_If* createComponent();

};

#endif
