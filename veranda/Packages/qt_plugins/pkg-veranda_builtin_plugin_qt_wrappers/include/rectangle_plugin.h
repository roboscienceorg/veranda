//! \file
#pragma once

#include <QObject>

#include "defines.h"

#include <veranda_qt_plugins/world_object_component_plugin.h>

/*!
 * \brief Plugin interface to the rectangle shape WorldObjectComponent
 */
class Rectangle_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID RECT_IID)
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    /*!
     * \brief Creates a new rectangle WorldObjectComponent
     * \return A newly allocated rectangle WorldObjectComponent
     */
    WorldObjectComponent *createComponent();

};
