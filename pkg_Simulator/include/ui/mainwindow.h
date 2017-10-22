#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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

    visualizerFactory _makeWidget;
    Simulator_Visual_If* _visual;

public:
    explicit MainWindow(visualizerFactory factory, QWidget *parent = 0);
    ~MainWindow();
    int speed;
    bool play;
    bool record;
    model_id _modelNum;

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

    void showMainWindow(){ show(); }
private slots:
    void simModeButtonClick();
    void mapModeButtonClick();
    void robotModeButtonClick();
    void showBuildObjectsButtonClick();
    void showMenuButtonClick();
    void playSimButtonClick();
    void speedSimButtonClick();
    void recordSimButtonClick();
    void chooseMapButtonClick();
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
