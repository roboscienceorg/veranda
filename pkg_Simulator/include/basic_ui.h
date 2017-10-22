#ifndef BASIC_UI_H
#define BASIC_UI_H

#include "interfaces/simulator_visual_if.h"
#include "interfaces/simulator_ui_if.h"

#include <functional>

class BasicUi : public Simulator_Ui_If
{
public:
    typedef std::function<Simulator_Visual_If*()> visualizerFactory;

private:
    Q_OBJECT

    visualizerFactory _visFactory;

public:
    BasicUi(visualizerFactory visualMaker, QWidget* parent = nullptr);

public slots:
    //Simulator core added a robot to simulation
    //Do not delete the robot interface when the robot is removed; it will be handled elsewhere
    void robotAddedToSimulation(Robot_Properties* robot) override;

    //Simulator core removed a robot from simulation
    void robotRemovedFromSimulation(robot_id rId) override;

    //A robot was selected as the 'current' robot
    void robotSelected(robot_id rId) override;

    //TODO: Need some way to specify which map to ui
    void mapSetInSimulation() override;

    //Slots to indicate that physics settings changed
    void physicsTickChanged(double rate_hz, double duration_s) override;
    void physicsStopped() override;
    void physicsStarted() override;

    //Slot to throw an error message to the user
    void errorMessage(QString error) override;

    virtual void showMainWindow() override;
};
#endif // BASIC_UI_H
