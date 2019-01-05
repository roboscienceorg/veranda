//! \file
#pragma once

#include <QObject>

#include "defines.h"

//This should not be fully defined relative; it should be just
//  <veranda_core/world_object_component_plugin.h>
//however, a bug? in MSVC prevents the Qt MOC from resolving interfaces
//if the path isn't relative like this and it doesn't seem to work to
//do conditional compilation so this would be used only on windows
#include "../../../../../../install/include/veranda_core_api/veranda_core/world_object_component_plugin.h"

/*!
 * \brief Plugin interface to provide the Ackermann Steering component
 */
class Ackermann_Steer_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID ACKERMANN_IID)
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    /*!
     * \brief Creates a new Ackermann Steering component
     * \return A newly allocated Ackermann Steering component
     */
    WorldObjectComponent* createComponent();

};
