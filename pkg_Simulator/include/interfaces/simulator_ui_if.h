#ifndef SIMULATOR_UI_H
#define SIMULATOR_UI_H

#include <functional>

#include <QObject>
#include <QVector>
#include <QWidget>
#include <QMainWindow>

#include <Box2D/Box2D.h>
#include <sdsmt_simulator/world_object.h>

#include "interfaces/world_object_wrappers.h"

class Simulator_Ui_If : public QMainWindow
{
    Q_OBJECT

public:
    Simulator_Ui_If(QWidget* parent = nullptr) : QMainWindow(parent){}

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

    //User requests that something be added to or removed from
    //the simulation
    void userAddWorldObjectsToSimulation(QVector<QSharedPointer<WorldObject>>);
    void userRemoveWorldObjectsFromSimulation(QVector<object_id>);

    void joystickMoved(double x, double y, double z, QString channel);
    void joystickButtonPress(int key, QString channel);
    void joystickButtonRelease(int key, QString channel);

public slots:
    //Simulator core added something to the simulation
    //Do not delete the world object when it is removed; that will be handled elsewhere
    virtual void worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>>) = 0;

    //Simulator core removed something from simulation
    virtual void worldObjectsRemovedFromSimulation(QVector<object_id> oId) = 0;

    //Slots to indicate that physics settings changed
    virtual void physicsTickChanged(double rate_hz, double duration_s) = 0;
    virtual void physicsStopped() = 0;
    virtual void physicsStarted() = 0;

    //Slot to throw an error message to the user
    virtual void errorMessage(QString error) = 0;

    virtual void showMainWindow() = 0;

    virtual void setWorldBounds(double xMin, double xMax, double yMin, double yMax) = 0;
};

#endif // SIMULATOR_UI_H
