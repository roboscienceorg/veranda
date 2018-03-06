#ifndef WORLD_OBJECT_COMPONENT_PLUGIN_H
#define WORLD_OBJECT_COMPONENT_PLUGIN_H

#include "world_object_component.h"
#include "dllapi.h"

class WorldObjectComponent_Plugin_If
{
public:
    virtual ~WorldObjectComponent_Plugin_If(){}

    virtual WorldObjectComponent* createComponent() = 0;
};

Q_DECLARE_INTERFACE(WorldObjectComponent_Plugin_If, "org.sdsmt.sim.2d.worldObjectComponent")

#endif
