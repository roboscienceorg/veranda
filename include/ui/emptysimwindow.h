#ifndef EMPTYSIMWINDOW_H
#define EMPTYSIMWINDOW_H

#include <QMainWindow>

#include "interfaces/simulator_ui_if.h"
#include "interfaces/robot_interfaces.h"
#include "interfaces/simulator_visual_if.h"

namespace Ui {
class emptysimwindow;
}

class emptysimwindow : public Simulator_Ui_If
{
public:
    typedef std::function<Simulator_Visual_If*()> visualizerFactory;

private:
    Q_OBJECT

    visualizerFactory _factory;
    robot_id _robots = 0;
    Simulator_Visual_If* _visual;

public:
    explicit emptysimwindow(visualizerFactory factory, QWidget *parent = 0);
    ~emptysimwindow();

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
    void physicsStopped(){}
    void physicsStarted(){}

    //Slot to throw an error message to the user
    void errorMessage(QString error){}

    void showMainWindow();

private:
    Ui::emptysimwindow *ui;
};

#endif // EMPTYSIMWINDOW_H
