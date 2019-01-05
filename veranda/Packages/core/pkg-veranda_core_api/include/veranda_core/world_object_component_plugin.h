//! \file
#pragma once

#include "veranda_core/world_object_component.h"
#include "veranda_core/dllapi.h"

/*!
 * \brief Plugin interface for plugins that provide new components
 * Almost all component types available in the application are provided
 * through plugins loaded at runtime; this is the plugin interface that
 * plugins should use to indicate that they provide a component
 */
class WorldObjectComponent_Plugin_If
{
public:
    //! Virtual destructor
    virtual ~WorldObjectComponent_Plugin_If(){}

    /*!
     * \brief Creates a new instance of the component this plugin adds
     * \return A new instance of the plugin's component
     */
    virtual WorldObjectComponent* createComponent() = 0;
};

Q_DECLARE_INTERFACE(WorldObjectComponent_Plugin_If, "org.roboscience.veranda.worldObjectComponent")
