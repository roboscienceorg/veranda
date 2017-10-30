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
    connect(_userInterface, &Simulator_Ui_If::userRemoveRobotFromSimulation, this, &SimulatorCore::removeSimRobot);
    connect(this, &SimulatorCore::robotRemoved, _userInterface, &Simulator_Ui_If::robotRemovedFromSimulation);
    connect(this, &SimulatorCore::robotRemoved, _physicsEngine, &Simulator_Physics_If::removeRobot);

    connect(_userInterface, &Simulator_Ui_If::userAddRobotIntoSimulation, this, &SimulatorCore::addSimRobotFromFile);
    connect(this, static_cast<void (SimulatorCore::*)(Robot_Physics*)>(&SimulatorCore::robotAdded), _physicsEngine, &Simulator_Physics_If::addRobot);
    connect(this, static_cast<void (SimulatorCore::*)(Robot_Properties*)>(&SimulatorCore::robotAdded), _userInterface, &Simulator_Ui_If::robotAddedToSimulation);

    connect(_userInterface, &Simulator_Ui_If::userSetMapInSimulation, this, &SimulatorCore::setSimMapFromFile);
    connect(this, &SimulatorCore::mapObjectsLoaded, _physicsEngine, &Simulator_Physics_If::newStaticShapes);

    connect(this, &SimulatorCore::userStartPhysics, _physicsEngine, &Simulator_Physics_If::start);
    connect(this, &SimulatorCore::userStopPhysics, _physicsEngine, &Simulator_Physics_If::stop);
    connect(this, &SimulatorCore::userSetPhysicsTick, _physicsEngine, &Simulator_Physics_If::setTick);

    connect(this, &SimulatorCore::physicsStarted, _userInterface, &Simulator_Ui_If::physicsStarted);
    connect(this, &SimulatorCore::physicsStopped, _userInterface, &Simulator_Ui_If::physicsStopped);
    connect(this, &SimulatorCore::physicsTickSet, _userInterface, &Simulator_Ui_If::physicsTickChanged);

    connect(_userInterface, &Simulator_Ui_If::userStartPhysics, this, &SimulatorCore::userStartPhysics);
    connect(_userInterface, &Simulator_Ui_If::userStopPhysics, this, &SimulatorCore::userStopPhysics);
    connect(_userInterface, &Simulator_Ui_If::userSetPhysicsTick, this, &SimulatorCore::userSetPhysicsTick);

    connect(_physicsEngine, &Simulator_Physics_If::physicsStarted, this, &SimulatorCore::physicsStarted);
    connect(_physicsEngine, &Simulator_Physics_If::physicsStopped, this, &SimulatorCore::physicsStopped);
    connect(_physicsEngine, &Simulator_Physics_If::physicsTickSet, this, &SimulatorCore::physicsTickSet);

    connect(this, &SimulatorCore::errorMsg, _userInterface, &Simulator_Ui_If::errorMessage);
}

SimulatorCore::~SimulatorCore()
{
    //Destroy all remaining robots
    while(_activeRobots.size())
        removeSimRobot(_activeRobots.firstKey());

    //Stop physics engine
    userStopPhysics();
    _physicsEngine->deleteLater();

    //Destroy ui
    _userInterface->deleteLater();

    delete _robotLoader;
    delete _mapLoader;
}

void SimulatorCore::start()
{
    //Set default physics tick rate
    userSetPhysicsTick(100.0, 1.0/100.0);

    //Show UI
    _userInterface->showMainWindow();

    //Add default robot
    addSimRobotFromFile("");

    for(auto iter = _activeRobots.begin(); iter != _activeRobots.end(); iter++)
    {
        //qDebug() << "Set channel property";
        iter.value()->getAllProperties()["Float Drive/channels/input_velocities"].set("robot0/world_velocity");
    }
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
        connect(_physicsEngine, &Simulator_Physics_If::physicsStarted, newBot, &Robot::connectToROS);
        connect(_physicsEngine, &Simulator_Physics_If::physicsStopped, newBot, &Robot::disconnectFromROS);

        Robot_Physics* phys_interface = new Robot_Physics(newBot, _nextRobotId);
        Robot_Properties* prop_interface = new Robot_Properties(newBot, _nextRobotId);

        //Send out robot interfaces
        emit robotAdded(phys_interface);
        emit robotAdded(prop_interface);

        //Keep references to robot and thread
        _activeRobots[_nextRobotId] = newBot;

        _nextRobotId++;

        if(_physicsEngine->running())
            newBot->connectToROS();
        else
            newBot->disconnectFromROS();
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

        //Delete robot object
        //This should delete all robot interfaces; they are children to it
        delete _activeRobots[rId];
        _activeRobots.remove(rId);
    }
}
