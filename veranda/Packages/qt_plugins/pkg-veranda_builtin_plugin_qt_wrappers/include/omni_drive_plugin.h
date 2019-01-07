//! \file
#pragma once

#include <QObject>

#include "defines.h"

#include <veranda_qt_plugins/world_object_component_plugin.h>

/*!
 * \brief Plugin interface to provide the Ackermann Steering component
 */
class Omni_Drive_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID OMNI_IID)
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    /*!
     * \brief Creates a new Ackermann Steering component
     * \return A newly allocated Ackermann Steering component
     */
    WorldObjectComponent* createComponent();
};
