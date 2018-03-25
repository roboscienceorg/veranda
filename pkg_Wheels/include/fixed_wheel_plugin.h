#ifndef FIXED_WHEEL_PLUGIN_H
#define FIXED_WHEEL_PLUGIN_H

#include <QObject>

#include "defines.h"

//This should not be fully defined relative; it should be just
//  <sdsmt_simulator/world_object_component_plugin.h>
//however, a bug? in MSVC prevents the Qt MOC from resolving interfaces
//if the path isn't relative like this and it doesn't seem to work to
//do conditional compilation so this would be used only on windows
#include "../../../install/include/sdsmt_simulator/world_object_component_plugin.h"

class Fixed_Wheel_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID FIXEDWHEEL_IID)
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    Fixed_Wheel_Plugin();
    WorldObjectComponent *createComponent();

};

#endif
