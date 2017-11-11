#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStandardItem>

#include "interfaces/simulator_ui_if.h"
#include "interfaces/robot_interfaces.h"
#include "interfaces/simulator_visual_if.h"


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

    int speed;
    bool play;
    bool record;
    model_id modelNum;
    model_id selected;
    QMap<model_id, Robot_Properties*> models;

    QMap<uint64_t, QString> displayed_properties;

public:
    explicit MainWindow(visualizerFactory factory, QWidget *parent = 0);
    ~MainWindow();

public slots:
    //Simulator core added a robot to simulation
    //Do not delete the robot interface when the robot is removed; it will be handled elsewhere
    void robotAddedToSimulation(Robot_Properties* robot);

    //Simulator core removed a robot from simulation
    void robotRemovedFromSimulation(robot_id rId){}

    //A robot was selected as the 'current' robot
    void robotSelected(robot_id rId){}

    //TODO: Need some way to specify which map to ui
    void mapSetInSimulation(){}

    //Slots to indicate that physics settings changed
    void physicsTickChanged(double rate_hz, double duration_s){}
    void physicsStopped();
    void physicsStarted();

    //Slot to throw an error message to the user
    void errorMessage(QString error){}

    //Slot to show main window
    void showMainWindow(){ show(); }

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
    void modelSelected(model_id id);
    void listBuildTools(int mode);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
