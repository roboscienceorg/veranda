#include "simulator_core.h"

#include <iostream>
#include <QDebug>
#include <QTimer>

using namespace std;

SimulatorCore::SimulatorCore(MapLoader_If *mapLoad, RobotLoader_If *robotLoad,
                                 Simulator_Physics_If *physics, Simulator_Ui_If *ui,
                                 QObject *parent) :
QObject(parent),
_mapLoader(mapLoad), _robotLoader(robotLoad),
_physicsEngine(physics), _userInterface(ui)
{
    _physicsThread = new QThread(this);
    _physicsEngine->moveToThread(_physicsThread);

    connect(_userInterface, &Simulator_Ui_If::userRemoveRobotFromSimulation, this, &SimulatorCore::removeSimRobot);
    connect(this, &SimulatorCore::robotRemoved, _userInterface, &Simulator_Ui_If::robotRemovedFromSimulation);
    connect(this, &SimulatorCore::robotRemoved, _physicsEngine, &Simulator_Physics_If::removeRobot);

    connect(_userInterface, &Simulator_Ui_If::userAddRobotIntoSimulation, this, &SimulatorCore::addSimRobotFromFile);
    connect(this, static_cast<void (SimulatorCore::*)(Robot_Physics*)>(&SimulatorCore::robotAdded), _physicsEngine, &Simulator_Physics_If::addRobot);
    connect(this, static_cast<void (SimulatorCore::*)(Robot_Properties*)>(&SimulatorCore::robotAdded), _userInterface, &Simulator_Ui_If::robotAddedToSimulation);

    connect(_userInterface, &Simulator_Ui_If::userSetMapInSimulation, this, &SimulatorCore::setSimMapFromFile);
    connect(this, &SimulatorCore::mapObjectsLoaded, _physicsEngine, &Simulator_Physics_If::newStaticShapes);

    connect(_userInterface, &Simulator_Ui_If::userStartPhysics, _physicsEngine, &Simulator_Physics_If::start);
    connect(_userInterface, &Simulator_Ui_If::userStopPhysics, _physicsEngine, &Simulator_Physics_If::stop);
    connect(_userInterface, &Simulator_Ui_If::userSetPhysicsTick, _physicsEngine, &Simulator_Physics_If::setTick);

    connect(this, &SimulatorCore::errorMsg, _userInterface, &Simulator_Ui_If::errorMessage);
}

SimulatorCore::~SimulatorCore()
{
    //Stop physics engine
    _physicsThread->quit();
    _physicsThread->wait();

    //Destroy all remaining robots
    while(_activeRobots.size())
        removeSimRobot(_activeRobots.firstKey());
}

void SimulatorCore::start()
{
    qInfo() << "Start physics engine";
    _physicsThread->start();

    qInfo() << "Show main window";
    _userInterface->showMainWindow();

    addSimRobotFromFile("");
    for(auto iter = _activeRobots.begin(); iter != _activeRobots.end(); iter++)
        QTimer::singleShot(10, iter.value(), &Robot::connectToROS);
}

void SimulatorCore::setSimMapFromFile(QString file)
{
    QVector<b2Shape*> mapShapes;
    QString error = _mapLoader->loadMapFile(file, mapShapes);
    if(!error.size())
    {
        emit mapObjectsLoaded(mapShapes);
    }
    else
    {
        emit errorMsg("Map " + file + " failed to load: " + error);
    }
}

void SimulatorCore::addSimRobotFromFile(QString file)
{
    Robot* newBot;
    QString error = _robotLoader->loadRobotFile(file, newBot);
    if(!error.size())
    {
        Robot_Physics* phys_interface = new Robot_Physics(newBot, _nextRobotId);
        Robot_Properties* prop_interface = new Robot_Properties(newBot, _nextRobotId);

        //Put robot in its own thread
        QThread* rThread = new QThread(this);
        newBot->moveToThread(rThread);

        //Send out robot interfaces
        emit robotAdded(phys_interface);
        emit robotAdded(prop_interface);

        //Keep references to robot and thread
        _activeRobots[_nextRobotId] = newBot;
        _robotThreads[_nextRobotId] = rThread;

        //Start robot's thread
        rThread->start();

        _nextRobotId++;
    }
    else
    {
        emit errorMsg("Robot " + file + " failed to load: " + error);
    }
}

void SimulatorCore::removeSimRobot(robot_id rId)
{
    if(_activeRobots.contains(rId))
    {
        //Signal that robot is no longer in simulation
        emit robotRemoved(rId);

        //End thread the robot was running on
        _robotThreads[rId]->quit();
        _robotThreads[rId]->wait();

        //Delete robot object
        //This should delete all robot interfaces; they are children to it
        delete _activeRobots[rId];
        _activeRobots.remove(rId);

        //Delete thread object
        delete _robotThreads[rId];
        _robotThreads.remove(rId);
    }
}
