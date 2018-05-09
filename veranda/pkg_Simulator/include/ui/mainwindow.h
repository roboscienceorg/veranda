//! \file
#pragma once

#include <QMainWindow>
#include <QStandardItem>
#include <QListWidgetItem>
#include <QProgressBar>
#include <QVector>
#include <QPair>
#include <QMap>

#include "interfaces/simulator_ui_if.h"
#include "interfaces/simulator_visual_if.h"
#include "interfaces/world_object_wrappers.h"
#include "ui/joystickprototype.h"
#include "ui/designer_widget.h"
#include "ui/mode_controller.h"

#include <veranda/world_object_component_plugin.h>
#include <veranda/object_loader_if.h>
#include <veranda/object_saver_if.h>

namespace Ui {
class MainWindow;
class Settings;
}

/*!
 * \brief The default User interface for the simulation
 *
 * The main UI fulfills the Simulator UI If interface and
 * provides a number of extra features.
 *
 * Features provided:
 * * Full WorldObject Designer
 * * Saving snapshots of the simulation
 * * Speed up/slow down the simulation
 * * Display and modify properties of selected object
 * * Load and save World Objects and full Simulations as files
 * * Quicksave/Quickload to reset simulation to previous state
 */
class MainWindow : public Simulator_Ui_If
{
public:
    //! Typedef for factory type to create view widgets
    typedef std::function<Simulator_Visual_If*()> visualizerFactory;

    /*!
     * \brief Creates a new instance of the ui
     * The functionality of the UI is dependent upon what plugins are found. The file types
     * that can be saved and loaded are determined by the loader/saver plugins (if none are found, the save and/or load
     * buttons are removed). If a default World Loader is provided, then the objects it creates will automatically be added
     * to every simulation.
     * \param[in] factory Factory for creating simulator visual widget
     * \param[in] components Map of the available WorldObjectComponent plugins; mapped by plugin IID
     * \param[in] oloaders List of world object loaders
     * \param[in] osavers List of world object savers
     * \param[in] wloaders List of simulation loaders
     * \param[in] wsavers List of simulation savers
     * \param[in] defaultLoader_ Loader to create default objects in all simulations
     * \param[in] parent QWidget parent
     */
    explicit MainWindow(visualizerFactory factory, QMap<QString, WorldObjectComponent_Plugin_If*> components,
                        QVector<WorldObjectLoader_If*> oloaders, QVector<WorldObjectSaver_If*> osavers,
                        QVector<WorldLoader_If*> wloaders, QVector<WorldSaver_If*> wsavers, WorldLoader_If *defaultLoader_=nullptr, QWidget *parent = 0);

    //! Cleans up the and destroys the UI
    ~MainWindow();
private:
    Q_OBJECT

    //! The visualization widget factory provided at construction
    visualizerFactory makeWidget;

    //! Widget collection for simulation mode
    Mode_Controller* simulator;

    //! Widget collection for designer mode
    Mode_Controller* designer;

    //! Map of available component plugins
    QMap<QString, WorldObjectComponent_Plugin_If*> componentPlugins;

    //! Map of single object loaders; mapped by file type
    QMap<QString, QVector<WorldObjectLoader_If*>> objectLoaders;

    //! Map of single object savers; mapped by file type
    QMap<QString, QVector<WorldObjectSaver_If*>> objectSavers;

    //! Map of simulation loaders; mapped by file type
    QMap<QString, QVector<WorldLoader_If*>> worldLoaders;

    //! Map of simulation savers; mapped by file type
    QMap<QString, QVector<WorldSaver_If*>> worldSavers;

    //! Map of objects currently in the simulation
    QMap<object_id, WorldObjectProperties*> objects;

    //! Default object loader, if provided
    WorldLoader_If* defaultLoader = nullptr;

    /*!
     * \brief Adds objects to the UI
     * Added objects are wrapped in the WorldObjectProperties wrapper, and paired with
     * an object_id
     * \param[in] objs The objects to add
     */
    void worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>> objs)
    {
        for(QPair<WorldObjectProperties*, object_id>& p : objs)
            objects[p.second] = p.first;

        objectsAddedToSimulation(objs);
    }

    /*!
     * \brief Removes objects from the UI
     * \param[in] oIds List of ids associated with the objects to be removed
     */
    void worldObjectsRemovedFromSimulation(QVector<object_id> oIds)
    {
        for(object_id i : oIds)
            objects.remove(i);

        objectsRemovedFromSimulation(oIds);
    }

    /*!
     * \brief Sets the bounds of what is viewable in the simulation
     * \param[in] xMin Minimum x coordinate to show
     * \param[in] xMax Maximum x coordinate to show
     * \param[in] yMin Minimum y coordinate to show
     * \param[in] yMax Maximum y coordinate to show
     */
    void setWorldBounds(double xMin, double xMax, double yMin, double yMax)
    {
        simulator->setWorldBounds(xMin, xMax, yMin, yMax);
    }

    //! Flag indicating if physics is running or not
    bool play = false;

    //! Index of current speed mode in SPEEDBUTTONS vector
    uint64_t speed = 1;

    //! List of available speed warp options and the images assocated with them
    const QVector<QPair<double, QPair<QString, QString>>> SPEEDBUTTONS
    {
        {1.0, {"Speed x1", ":/sim/SpeedOne"}},
        {2.0, {"Speed x2", ":/sim/SpeedTwo"}},
        {3.0, {"Speed x3", ":/sim/SpeedThree"}},
        {0.5, {"Speed x1/2", ":/sim/SpeedHalf"}}
    };

public slots:
    //Slots to indicate that physics settings changed
    void physicsTickChanged(double rate_hz, double duration_s){}
    void physicsTickMultiplierChanged(double mult);
    void physicsStopped();
    void physicsStarted();

    //Slot to throw an error message to the user
    void errorMessage(QString error);

    //Slot to show main window
    void showMainWindow(){
        show();
    }

    /*!
     * \brief Captures event on window close to forward it in a signal
     * \param[in] e The close event
     */
    void closeEvent(QCloseEvent *e)
    {
        emit windowClosed();
    }

private slots:

    //! Switches to simulation mode
    void simulatorButtonClick();

    //! Switches to designer mode
    void designerButtonClick();

    //! Toggles the toolbox pane
    void showBuildObjectsButtonClick();

    //! Toggles the general menu (left side) pane
    void showMenuButtonClick();

    //! Starts/Stops the simulation
    void playSimButtonClick();

    //! Changes to the next time warp speed
    void speedSimButtonClick();

    //! Saves a screenshot of the simulation
    void screenshotSimButtonClick();

    //! Clears the simulation and loads a new one
    void loadSimButtonClick();

    //! Opens a new joystick window
    void joystickButtonClick();

    //! Saves the state of the simulation
    void saveSimButtonClick();

    //! Creates a quicksave file of the simulation
    void quickSaveButtonClick();

    //! Loads the last quicksave file of the simulation
    void quickLoadButtonClick();

    //! Clears the designer view
    void newObjectButtonClick();

    //! Clears the simulation
    void newSimButtonClick();

    //! Adds an item to the designer
    void loadObjectButtonClick();

    //! Saves all components in the designer as an object
    void saveObjectButtonClick();

    //! Loads an object file for the simulator toolbox
    void loadObjectsForSimButtonClick();

    //! Exports the current set of components in the designer to the simulator toolbox as an object
    void exportObjectButtonClick();

    //! Builds the toolbox for the designer
    void loadToolsButtonClick();

private:
    //! Autogenerated UI class
    Ui::MainWindow *ui;

signals:
    /*! \brief Signals which object is 'selected'
     * \param id The object_id of the selected object
     */
    void objectIsSelected(object_id id);

    //! Signals that no object is 'selected'
    void nothingIsSelected();

    //! Signals that the window is closing
    void windowClosed();

    /*!
     * \brief Forwarding signal when objects are added to simulator
     * \param[in] objects The list of objects added to the simulation
     */
    void objectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>> objects);

    /*!
     * \brief Forwarding signal when objects are removed from simulator
     * \param[in] ids The ids of objects to be removed
     */
    void objectsRemovedFromSimulation(QVector<object_id> ids);

    //void addObjectToSimulation(QVector<QSharedPointer<WorldObject>>);

    //tools menu (rightmost deployable menu)
    //void addToolToSimulator(WorldObjectProperties* properties);
    //void deleteToolFromSimulator(WorldObjectProperties* properties);
    /*!
     * \brief Signal an error to be shown
     * \param[in] err The error message to display
     */
    void error(QString err);
};

