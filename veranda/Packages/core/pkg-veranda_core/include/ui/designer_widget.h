//! \file
#pragma once

#include <QWidget>
#include <QListWidgetItem>

#include "interfaces/simulator_ui_if.h"
#include "interfaces/simulator_visual_if.h"
#include "interfaces/world_object_wrappers.h"

#include <veranda_core/world_object_component_plugin.h>

/*!
 * \brief Container for an item in one of the UI toolboxes
 * Both the simulation view and designer view have a toolbox widget which
 * shows the objects and object components that can be put on screen; this
 * widget is a custom list item for displaying those options along with a
 * picture of their models.
 */
class Designer_Widget : public QListWidgetItem
{
    //! Typedef for the factory type which makes the visuals
    typedef std::function<Simulator_Visual_If*()> visualizerFactory;

public:
    //! The view of the held objects models
    Simulator_Visual_If* view;

    //! The object represented if it is a component
    WorldObjectComponent* component;

    //! The properties object of the item represented on the list
    WorldObjectProperties* properties;

    /*!
     * \brief Constructs a new list widget item for something that can be added to the screen
     * \param[in] object The component represented in this widget
     * \param[in] object2 The properties of the item represented
     * \param[in] factory Factory for making the view of the item
     * \param[in] parent QListWidget this is an item in
     * \param[in] simulator Whether or not this widget item is used for the simulation or the designer
     */
    Designer_Widget(WorldObjectComponent *object, WorldObjectProperties *object2, visualizerFactory factory, QListWidget *parent=nullptr, bool simulator = false);

private:
    //there will be x amount of these widgets, each one for a different TAB of the Designer visual widget lists
};
