#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStandardItem>

#include "interfaces/simulator_ui_if.h"
#include "interfaces/simulator_visual_if.h"
#include "interfaces/map_loader_if.h"
#include "interfaces/robot_loader_if.h"

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
    Simulator_Visual_If* visual;

    MapLoader_If* mapLoader;
    RobotLoader_If* robotLoader;

    int speed;
    bool play;
    bool record;
    object_id selected;

    QStandardItemModel* propertiesModel = nullptr;

    QMap<object_id, WorldObjectProperties_If*> worldObjects;

    QMap<uint64_t, QString> displayed_properties;

public:
    explicit MainWindow(visualizerFactory factory, MapLoader_If* mapLoad, RobotLoader_If* robotLoad, QWidget *parent = 0);
    ~MainWindow();

public slots:
    //Simulator core added something to the simulation
    //Do not delete the world object when it is removed; that will be handled elsewhere
    virtual void worldObjectAddedToSimulation(WorldObjectProperties_If* object, object_id oId);

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

        qDebug() << "Populating default robots...";
        for(int i=0; i<3; i++)
        {
            Robot* r = robotLoader->loadRobotFile("");
            if(r)
            {
                r->getAllProperties()["Diff Drive/channels/input_velocities"].set("robot0/wheel_velocities");
                r->getAllProperties()["Diff Drive/axle_length"].set(1);
                r->getAllProperties()["Diff Drive/wheel_radius"].set(0.2);
                userAddWorldObjectToSimulation(r);
                delete r;
            }
        }
    }

private slots:

    //Slots for button clicks on all menus
    void simModeButtonClick();
    void mapModeButtonClick();
    void robotModeButtonClick();
    void showBuildObjectsButtonClick();
    void showMenuButtonClick();
    void playSimButtonClick();
    void speedSimButtonClick();
    void recordSimButtonClick();
    void importMapButtonClick();

    //Slots for build tools and properties
    void objectSelected(object_id id);
    void nothingSelected();
    void listBuildTools(int mode);

private:
    Ui::MainWindow *ui;

signals:
    void objectIsSelected(object_id id);
    void nothingIsSelected();
};

#endif // MAINWINDOW_H
