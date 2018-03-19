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
#include "ui/mode_controller.h"

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

    explicit MainWindow(visualizerFactory factory, QMap<QString, WorldObjectComponent_Plugin_If*> components,
                        QVector<WorldObjectLoader_If*> oloaders, QVector<WorldObjectSaver_If*> osavers,
                        QVector<WorldLoader_If*> wloaders, QVector<WorldSaver_If*> wsavers, WorldLoader_If *defaultLoader_=nullptr, QWidget *parent = 0);
    ~MainWindow();
private:
    Q_OBJECT

    visualizerFactory makeWidget;
    Mode_Controller* simulator;
    Mode_Controller* designer;

    QMap<QString, WorldObjectComponent_Plugin_If*> componentPlugins;
    QMap<QString, QVector<WorldObjectLoader_If*>> objectLoaders;
    QMap<QString, QVector<WorldObjectSaver_If*>> objectSavers;

    QMap<QString, QVector<WorldLoader_If*>> worldLoaders;
    QMap<QString, QVector<WorldSaver_If*>> worldSavers;

    QMap<object_id, WorldObjectProperties*> objects;
    WorldLoader_If* defaultLoader = nullptr;

    void worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>> objs)
    {
        for(QPair<WorldObjectProperties*, object_id>& p : objs)
            objects[p.second] = p.first;

        objectsAddedToSimulation(objs);
    }
    void worldObjectsRemovedFromSimulation(QVector<object_id> oId)
    {
        for(object_id i : oId)
            objects.remove(i);

        objectsRemovedFromSimulation(oId);
    }
    void setWorldBounds(double xMin, double xMax, double yMin, double yMax){}

    bool play = false;
    bool record;
    uint64_t speed = 1;

    const QVector<QPair<double, QPair<QString, QString>>> SPEEDBUTTONS
    {
        {1.0, {"Speed x1", ":/sim/SpeedOneSimIcon"}},
        {2.0, {"Speed x2", ":/sim/SpeedTwoSimIcon"}},
        {3.0, {"Speed x3", ":/sim/SpeedThreeSimIcon"}},
        {0.5, {"Speed x1/2", ":/sim/SpeedHalfSimIcon"}}
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
    void loadSimButtonClick();
    void joystickButtonClick();
    void saveSimButtonClick();
    void restartSimButtonClick();

    //Designer mode button signals and slots
    void newObjectButtonClick();
    void loadObjectButtonClick();
    void saveObjectButtonClick();

    //RIGHT HAND MENU - Tool button signals and slots
    void loadObjectsForSimButtonClick();
    void exportObjectButtonClick();
    void loadToolsButtonClick();

private:
    Ui::MainWindow *ui;

signals:
    //selection slots for clicks on the world view
    void objectIsSelected(object_id);
    void nothingIsSelected();
    void windowClosed();

    void objectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>>);
    void objectsRemovedFromSimulation(QVector<object_id>);

    //void addObjectToSimulation(QVector<QSharedPointer<WorldObject>>);

    //tools menu (rightmost deployable menu)
    //void addToolToSimulator(WorldObjectProperties* properties);
    //void deleteToolFromSimulator(WorldObjectProperties* properties);
    void error(QString);
};

#endif // MAINWINDOW_H

