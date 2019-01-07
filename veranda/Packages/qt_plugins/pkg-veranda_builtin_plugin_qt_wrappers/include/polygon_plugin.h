//! \file
#pragma once

#include <QObject>

#include "defines.h"

#include <veranda_qt_plugins/world_object_component_plugin.h>

/*!
 * \brief Plugin interface to the circle shape WorldObjectComponent
 */
class Polygon_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID POLYGON_IID)
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    /*!
     * \brief Creates a new polygon WorldObjectComponent
     * \return A newly allocated polygon WorldObjectComponent
     */
    WorldObjectComponent *createComponent();

};
