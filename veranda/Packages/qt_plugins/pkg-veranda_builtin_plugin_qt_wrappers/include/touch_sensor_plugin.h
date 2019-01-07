//! \file
#pragma once

#include <QObject>

#include "defines.h"

#include <veranda_qt_plugins/world_object_component_plugin.h>

/*!
 * \brief Plugin interface to the touch sensor ring WorldObjectComponent
 */
class Touch_Sensor_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID TOUCH_IID)
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    /*!
     * \brief Creates a new sensor ring WorldObjectComponent
     * \return A newly allocated circle WorldObjectComponent
     */
    WorldObjectComponent* createComponent();

};
