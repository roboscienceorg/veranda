//! \file
#pragma once

#include <QObject>

#include "defines.h"

#include <veranda_qt_plugins/world_object_component_plugin.h>

/*!
 * \brief Plugin interface for the fixed wheel WorldObjectComponent
 */
class Swedish_Wheel_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID SWEDISHWHEEL_IID)
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    /*!
     * \brief Creates a new fixed wheel component
     * \return A newly constructed Fixed Wheel component
     */
    WorldObjectComponent *createComponent();

};