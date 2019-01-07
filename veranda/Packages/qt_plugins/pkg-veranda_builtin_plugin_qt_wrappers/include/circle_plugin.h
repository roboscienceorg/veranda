//! \file
#pragma once

#include <QObject>

#include "defines.h"

#include <veranda_qt_plugins/world_object_component_plugin.h>

/*!
 * \brief Plugin interface to the circle shape WorldObjectComponent
 */
class Circle_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID CIRCLE_IID)
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    /*!
     * \brief Creates a new circle WorldObjectComponent
     * \return A newly allocated circle WorldObjectComponent
     */
    WorldObjectComponent *createComponent();

};
