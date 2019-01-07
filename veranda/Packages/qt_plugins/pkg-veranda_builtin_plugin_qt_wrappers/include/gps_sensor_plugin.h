//! \file
#pragma once

#include <QObject>

#include "defines.h"

#include <veranda_qt_plugins/world_object_component_plugin.h>

/*!
 * \brief Plugin interface to provide gps components
 */
class GPS_Sensor_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID GPS_IID)
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    /*!
     * \brief Creates a new gps component
     * \return A newly construted Gps Component
     */
    WorldObjectComponent* createComponent();

};
