//! \file
#pragma once

#include <QStandardItem>
#include <QListWidgetItem>
#include <QHeaderView>
#include <QProgressBar>
#include <QVector>
#include <QPair>
#include <QTableView>
#include <QToolButton>
#include <QMap>

#include "interfaces/simulator_ui_if.h"
#include "interfaces/simulator_visual_if.h"
#include "interfaces/world_object_wrappers.h"
#include "ui/joystickprototype.h"
#include "ui/designer_widget.h"

#include <veranda_core/world_object_component_plugin.h>
#include <veranda_core/object_loader_if.h>
#include <veranda_core/object_saver_if.h>

/*!
 * \brief Manager for the different modes in the UI
 * Designer mode and Simulator mode share a lot of common functionality - Having a toolbar,
 * being able to add things from to toolbar to the view, being able to select items in that view
 * to see and edit properties, etc.
 *
 * This class manages all of those common functions
 */
class Mode_Controller: public QObject
{
    Q_OBJECT

    //! Typedef for the factory type to create the drawing canvas
    typedef std::function<Simulator_Visual_If*()> visualizerFactory;

public:
    /*!
     * \brief Mode_Controller Constructs a mode controller with a set of widgets to link and manage
     * \param[in] factory Factory object to create the visual
     * \param[in] pModeButton Button to activate this mode
     * \param[in] pMenu Menu bar to show in this mode
     * \param[in] pToolsMenu Toolbar to show in this mode
     * \param[in] pActive List of active tools for the widget
     * \param[in] pProperties Properties list widget this mode will use
     * \param[in] pTabs Tab widget for toolbar in this mode
     * \param[in] parent Parent QWidget
     */
    Mode_Controller(visualizerFactory factory, QToolButton* pModeButton, QWidget* pMenu, QWidget* pToolsMenu, QListWidget* pActive, QTableView* pProperties, QTabWidget* pTabs, QWidget *parent = nullptr);

    //! Objects visible in this mode
    QMap<object_id, WorldObjectProperties*> worldObjects;

    //! Visual for showing objects in this mode
    Simulator_Visual_If* visual;

    //! Whether or not this mode is the simulation mode
    bool simulator = false;

private:
    //! Counter for assigning object ids in designer
    object_id idIncrementer = 1;

    //! The button to switch to this mode
    QToolButton* modeButton;

    //! The menu bar
    QWidget* menu;

    //! The tool bar
    QWidget* toolsMenu;

    //! Toobar list of items
    QListWidget* active;

    //! Tab widget for toolbar groups
    QTabWidget* tabs;

    //! List of what goes in each tool tab
    QMap<QString, QListWidget*> toolTabs;

    //! Factory to make view widgets
    visualizerFactory makeWidget;

    //! Current selected object
    object_id selected;

    //! Selected toolbar item
    WorldObjectProperties* selectedTool = nullptr;

    //! View for properties list
    QTableView* properties;

    //! Model for properties list
    QStandardItemModel* propertiesModel;

    //! Items in list view for toolbar
    QMap<object_id, QListWidgetItem*> listItems;

    //! Properties of selected item (the ones shown in the properties table)
    QMap<QString, QSharedPointer<PropertyView>> selectedProps;

    //! Mapping of int index to property name of the properties in the table view
    QMap<uint64_t, QString> displayed_properties;

    /*!
     * \brief Determines which toolbar item is selected and makes a copy of the component it represents
     * The copied item is stored in selectedTool
     * \return True for success, false for fail
     */
    bool cloneSelectedTool();

public slots:
    /*!
     * \brief Handles simulation items being added through a signal from the simulator core
     * \param objs List of objects added and their associated object_ids
     */
    virtual void worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>> objs);

    /*!
     * \brief Handles simulation items being removed through a signal from the simulator core
     * \param oIds Ids associated with the objects being removed
     */
    virtual void worldObjectsRemovedFromSimulation(QVector<object_id> oIds);

    //! Creates a new model for the properties table
    void setPropertiesTableView();

    //! Activate the mode
    void open();

    //! Deactivate the mode
    void close();

    //! Clear all objects added in the mode
    void clear();

    //! Gets an unused id that can be assigned to a new item
    int getNextId();

    //! Adds the currently selected toolbar item to the view
    void addObjectToView();

    /*!
     * \brief Gets the set of components that are currently displayed in the view
     * \return List of WorldObjectComponent*
     */
    QVector<WorldObjectComponent*> getComponents();

    //! Removes the selected item from the view or signals a request to remove it
    void deleteObjectFromView();

    /*!
     * \brief Adds an option to the toolbar list
     * \param[in] component The item to add to the tools list
     */
    void addObjectToTools(WorldObjectComponent* component);

    /*!
     * \brief Marks an item selected and causes it to be drawn to indicate so
     * \param[in] id Id of the item to mark selected
     */
    void objectSelected(object_id id);

    //! Marks all items a unselected
    void nothingSelected();

    /*!
     * \brief Selects something on the view because it was clicked on the list of all components
     * \param item The item clicked on the list
     */
    void robotItemClicked(QListWidgetItem* item);

    //! Fills out the properties table based on the currently selected item
    void updatePropertyInformation();

    /*!
     * \brief Handles a drag-move event on the drawing canvas
     * \param[in] id Id of the item being dragged
     * \param[in] dx Distance dragged horizontally
     * \param[in] dy Distance dragged vertically
     */
    void simObjectMoveDragged(object_id id, double dx, double dy);

    /*!
     * \brief Handles a drag-rotate event on the drawing canvas
     * \param[in] id Id of the item being dragged
     * \param[in] dt Distance rotated (degrees)
     */
    void simObjectRotateDragged(object_id id, double dt);

signals:
    /*!
     * \brief Requests that world objects be added to the simulation
     * \param[in] objects Objects to be added to the simulation
     * \param[in] copy Whether or not the simulator core should copy the objects or take ownership of them
     */
    void requestAddWorldObjects(QVector<WorldObject*> objects, bool copy);

    /*!
     * \brief Requests that world objects be removed from the simulation
     * \param[in] ids object_ids associated with the objects that should be removed
     */
    void requestRemoveWorldObjects(QVector<object_id> ids);

    /*!
     * \brief Signals that the user clicked to select an item
     * \param[in] id object_id of the component that was clicked
     */
    void objectIsSelected(object_id id);

    /*!
     * \brief Signals that all components should be deselected
     */
    void nothingIsSelected();
};
