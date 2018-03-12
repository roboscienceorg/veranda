#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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

#include <sdsmt_simulator/world_object_component_plugin.h>
#include <sdsmt_simulator/object_loader_if.h>
#include <sdsmt_simulator/object_saver_if.h>

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

    visualizerFactory factory;
    visualizerFactory makeWidget;
    Simulator_Visual_If* visualSimulator;
    Simulator_Visual_If* visualDesigner;

    QMap<QString, WorldObjectComponent_Plugin_If*> componentPlugins;

    //tools tab widget holders, each list belongs to a tab
    QMap<QString, QListWidget*> designerTabs;
    QMap<QString, QListWidget*> simulatorTabs;

    bool simulation;

    QMap<QString, QVector<WorldObjectLoader_If*>> objectLoaders;
    QMap<QString, QVector<WorldObjectSaver_If*>> objectSavers;

    QMap<QString, QVector<WorldLoader_If*>> worldLoaders;
    QMap<QString, QVector<WorldSaver_If*>> worldSavers;
    WorldLoader_If* defaultLoader = nullptr;

    bool play;
    bool record;
    uint64_t speed = 0;

    object_id selected;
    const QVector<QPair<double, QPair<QString, QString>>> SPEEDBUTTONS
    {
        {1.0, {"Speed x1", ":/sim/SpeedOneSimIcon"}},
        {2.0, {"Speed x2", ":/sim/SpeedTwoSimIcon"}},
        {3.0, {"Speed x3", ":/sim/SpeedThreeSimIcon"}},
        {0.5, {"Speed x1/2", ":/sim/SpeedHalfSimIcon"}}
    };

    QStandardItemModel* propertiesModel = nullptr;

    //simulator objects
    QMap<object_id, WorldObjectProperties*> worldObjects;
    QMap<object_id, QListWidgetItem*> listItems;
    QMap<QString, QSharedPointer<PropertyView>> selectedProps;

    //designer objects
    QMap<object_id, WorldObjectProperties*> designerObjects;
    QMap<object_id, QListWidgetItem*> designerItems;

    QMap<uint64_t, QString> displayed_properties;

public:
    explicit MainWindow(visualizerFactory factory, QMap<QString, WorldObjectComponent_Plugin_If*> components,
                        QVector<WorldObjectLoader_If*> oloaders, QVector<WorldObjectSaver_If*> osavers,
                        QVector<WorldLoader_If*> wloaders, QVector<WorldSaver_If*> wsavers, WorldLoader_If *defaultLoader_=nullptr, QWidget *parent = 0);
    ~MainWindow();

public slots:
    //Simulator core added something to the simulation
    //Do not delete the world object when it is removed; that will be handled elsewhere
    virtual void worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>> objs);

    //Simulator core removed something from simulation
    virtual void worldObjectsRemovedFromSimulation(QVector<object_id> oIds);

    //Slots to indicate that physics settings changed
    void physicsTickChanged(double rate_hz, double duration_s){}
    void physicsTickMultiplierChanged(double mult);
    void physicsStopped();
    void physicsStarted();

    //Slot to throw an error message to the user
    void errorMessage(QString error);

    void setWorldBounds(double xMin, double xMax, double yMin, double yMax);

    //Slot to show main window
    void showMainWindow(){
        show();

        setWorldBounds(-100, 100, -100, 100);
    }

    void closeEvent(QCloseEvent *)
    {
        emit windowClosed();
    }

    void simObjectMoveDragged(object_id id, double dx, double dy);
    void simObjectRotateDragged(object_id id, double dt);
    void buildObjectMoveDragged(object_id id, double dx, double dy);
    void buildObjectRotateDragged(object_id id, double dt);

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
    void addObjectToSimButtonClick();
    void deleteObjectFromSimButtonClick();
    void loadObjectsForSimButtonClick();

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
    void addToolToSimulator(WorldObjectProperties* properties);
    void deleteToolFromSimulator(WorldObjectProperties* properties);
    void error(QString);
};

#endif // MAINWINDOW_H
