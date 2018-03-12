#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStandardItem>
#include <QListWidgetItem>

#include "interfaces/simulator_ui_if.h"
#include "interfaces/simulator_visual_if.h"
#include "interfaces/world_object_wrappers.h"
#include "ui/joystickprototype.h"
#include "ui/designer_widget.h"

#include <sdsmt_simulator/world_object_component_plugin.h>
#include <sdsmt_simulator/world_object_loader_if.h>
#include <sdsmt_simulator/world_object_saver_if.h>

namespace Ui {
class MainWindow;
class Settings;
}

class MainWindow : public Simulator_Ui_If
{
public:
    typedef std::function<Simulator_Visual_If*()> visualizerFactory;

private:
    Q_OBJECT

    visualizerFactory makeWidget;
    Simulator_Visual_If* visualSimulator;
    Simulator_Visual_If* visualDesigner;

    QMap<QString, WorldObjectComponent_Plugin_If*> componentPlugins;
    QMap<QString, WorldObjectLoader_If*> objectLoaders;
    QMap<QString, WorldObjectSaver_If*> objectSavers;

    //tools tab widget holders, each list belongs to a tab
    QMap<QString, QListWidget*> designerTabs;
    QMap<QString, QListWidget*> simulatorTabs;

    int speed;
    bool play;
    bool record;
    bool simulation;
    object_id selected;

    QStandardItemModel* propertiesModel = nullptr;

    //simulator objects
    QMap<object_id, WorldObjectProperties*> worldObjects;
    QMap<object_id, QListWidgetItem*> listItems;

    //designer objects
    QMap<object_id, WorldObjectProperties*> designerObjects;
    QMap<object_id, QListWidgetItem*> designerItems;

    QMap<uint64_t, QString> displayed_properties;

public:
    explicit MainWindow(visualizerFactory factory, QMap<QString, WorldObjectComponent_Plugin_If*> components,
                        QVector<WorldObjectLoader_If*> loaders, QVector<WorldObjectSaver_If*> savers, QWidget *parent = 0);
    ~MainWindow();

public slots:
    //Simulator core added something to the simulation
    //Do not delete the world object when it is removed; that will be handled elsewhere
    virtual void worldObjectAddedToSimulation(WorldObjectProperties* object, object_id oId);

    //Simulator core removed something from simulation
    virtual void worldObjectRemovedFromSimulation(object_id oId);

    //Slots to indicate that physics settings changed
    void physicsTickChanged(double rate_hz, double duration_s){}
    void physicsStopped();
    void physicsStarted();

    //Slot to throw an error message to the user
    void errorMessage(QString error){}

    void setWorldBounds(double xMin, double xMax, double yMin, double yMax);

    //Slot to show main window
    void showMainWindow(){
        show();

        setWorldBounds(-30, 30, -30, 30);
    }

    void closeEvent(QCloseEvent *)
    {
        emit windowClosed();
    }

private slots:

    //Main menu signals and slots
    void simulatorButtonClick();
    void designerButtonClick();
    void showBuildObjectsButtonClick();
    void showMenuButtonClick();

    //Simulation mode button signals and slots
    void playSimButtonClick();
    void speedSimButtonClick();
    void screenshotSimButtonClick();
    void importMapButtonClick();
    void joystickButtonClick();
    void saveSimButtonClick();
    void restartSimButtonClick();

    //Designer mode button signals and slots
    void newObjectButtonClick();
    void loadObjectButtonClick();
    void saveObjectButtonClick();

    //Simulation mode tool button signals and slots
    void addObjectButtonClick();
    void deleteObjectButtonClick();
    void loadObjectButtonClick();

    //Designer mode tool button signals and slots
    void addToolButtonClick();
    void deleteToolButtonClick();
    void exportObjectButtonClick();
    void loadToolsButtonClick();

    //World view signals and slots
    void objectSelected(object_id id);
    void nothingSelected();
    void robotItemClicked(QListWidgetItem* item);
    void updatePropertyInformation();

private:
    Ui::MainWindow *ui;

signals:
    //selection slots for clicks on the world view
    void objectIsSelected(object_id id);
    void nothingIsSelected();
    void windowClosed();

    //tools menu (rightmost deployable menu)
    void addToolToSimulator(WorldObjectProperties*);
    void addToolToDesigner(WorldObjectProperties*);
    void deleteToolFromSimulator(WorldObjectProperties*);
    void deleteToolFromDesigner(WorldObjectProperties*);
};

#endif // MAINWINDOW_H
