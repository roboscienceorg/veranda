#ifndef SIMULATOR_UI_H
#define SIMULATOR_UI_H

#include <functional>

#include <QObject>
#include <QVector>

#include <Box2D/Box2D.h>

#include "robot_interfaces.h"

class Simulator_Ui_If : public QObject
{
    Q_OBJECT

public:
    Simulator_Ui_If(QObject* parent = nullptr) : QObject(parent){}

signals:
    //Signals for simulator window open/close
    void simulatorWinOpened();
    void simulatorWinClosed();

    //Signals for map editor window open/close
    void mapWinOpened();
    void mapWinClosed();

    //Signals for robot editor window open/close
    void robotWinOpened();
    void robotWinClosed();

    //Signals for user options to change simulation ticks
    //If these go through successfully, the corresponding slots
    //will get called
    void userSetPhysicsTick(double rate_hz, double duration_s);
    void userStopPhysics();
    void userStartPhysics();

    //Signal that the user wants to create a robot from file
    //and add it
    //If the robot is successfully added, this object will
    //have a call to it's robotAddedToSimulation() slot
    void userAddRobotIntoSimulation(QString robotFilePath);

    //Signal that the user wants a robot removed from
    //simulation
    //If the robot is successfully removed, this object
    //will have a call to its robotRemovedFromSimulation() slot
    void userRemoveRobotFromSimulation(robot_id rId);

    //Signal that the user switched the simulation map
    void userSetMapInSimulation(QString mapFilePath);

    //Signal that they user wants to view properties
    //for a robot
    void userSelectedRobot(robot_id rId);

public slots:
    //Simulator core added a robot to simulation
    //Do not delete the robot interface when the robot is removed; it will be handled elsewhere
    virtual void robotAddedToSimulation(Robot_Properties* robot) = 0;

    //Simulator core removed a robot from simulation
    virtual void robotRemovedFromSimulation(robot_id rId) = 0;

    //A robot was selected as the 'current' robot
    virtual void robotSelected(robot_id rId) = 0;

    //TODO: Need some way to specify which map to ui
    virtual void mapSetInSimulation() = 0;

    //Slots to indicate that physics settings changed
    virtual void physicsTickChanged(double rate_hz, double duration_s) = 0;
    virtual void physicsStopped() = 0;
    virtual void physicsStarted() = 0;

    //Slot to throw an error message to the user
    virtual void errorMessage(QString error) = 0;

    virtual void showMainWindow() = 0;
};

#endif // SIMULATOR_UI_H
