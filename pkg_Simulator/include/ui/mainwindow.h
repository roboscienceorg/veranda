#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStandardItem>
#include <QListWidgetItem>
#include <QProgressBar>
#include <QVector>
#include <QPair>

#include "interfaces/simulator_ui_if.h"
#include "interfaces/simulator_visual_if.h"
#include "interfaces/world_object_wrappers.h"
//#include "interfaces/joystickprototype_if.h"
#include "ui/joystickprototype.h"

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

    visualizerFactory makeWidget;
    Simulator_Visual_If* visualSimulator;
    Simulator_Visual_If* visualDesigner;
    //QMap<QWindow*, JoystickPrototype*> joysticks;

    QMap<QString, WorldObjectComponent_Plugin_If*> componentPlugins;

    QMap<QString, QVector<WorldObjectLoader_If*>> objectLoaders;
    QMap<QString, QVector<WorldObjectSaver_If*>> objectSavers;

    QMap<QString, QVector<WorldLoader_If*>> worldLoaders;
    QMap<QString, QVector<WorldSaver_If*>> worldSavers;

    int speed;
    bool play;
    bool record;
    object_id selected;

    QStandardItemModel* propertiesModel = nullptr;

    QMap<object_id, WorldObjectProperties*> worldObjects;
    QMap<object_id, QListWidgetItem*> listItems;
    QMap<QString, QSharedPointer<PropertyView>> selectedProps;

    QMap<uint64_t, QString> displayed_properties;

public:
    explicit MainWindow(visualizerFactory factory, QMap<QString, WorldObjectComponent_Plugin_If*> components,
                        QVector<WorldObjectLoader_If*> oloaders, QVector<WorldObjectSaver_If*> osavers,
                        QVector<WorldLoader_If*> wloaders, QVector<WorldSaver_If*> wsavers, QWidget *parent = 0);
    ~MainWindow();

public slots:
    //Simulator core added something to the simulation
    //Do not delete the world object when it is removed; that will be handled elsewhere
    virtual void worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>> objs);

    //Simulator core removed something from simulation
    virtual void worldObjectsRemovedFromSimulation(QVector<object_id> oIds);

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

        //setWorldBounds(-200, 200, -200, 200);
        setWorldBounds(-50, 50, -50, 50);
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
    void addObjectButtonClick();
    void deleteObjectButtonClick();

    //Designer mode tool button signals and slots
    void addToolButtonClick();
    void deleteToolButtonClick();
    void exportObjectButtonClick();

    //World view signals and slots
    void objectSelected(object_id id);
    void nothingSelected();
    void updateDesignerBuildTools();
    void updateSimulatorBuildTools();
    void robotItemClicked(QListWidgetItem* item);
    void updatePropertyInformation();

private:
    Ui::MainWindow *ui;

signals:
    void objectIsSelected(object_id id);
    void nothingIsSelected();
    void windowClosed();
};

#endif // MAINWINDOW_H
