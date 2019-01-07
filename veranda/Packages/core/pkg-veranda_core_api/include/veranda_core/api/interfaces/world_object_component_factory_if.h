#pragma once

#include "veranda_core/api/world_object_component.h"

class WorldObjectComponent_Factory_If
{
    public:
    /*!
     * \brief Creates a new instance of a component
     * \return A new instance of some component
     */
    virtual WorldObjectComponent* createComponent() = 0;
};