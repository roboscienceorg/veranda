#ifndef TOUCH_SENSOR_PLUGIN_H
#define TOUCH_SENSOR_PLUGIN_H

#include <QObject>

#include <sdsmt_simulator/world_object_component_plugin.h>

class Touch_Sensor_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.sdsmt.sim.2d.worldObjectComponent.defaults.touchring")
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    Touch_Sensor_Plugin();
    WorldObjectComponent_If* createComponent();

};

#endif
