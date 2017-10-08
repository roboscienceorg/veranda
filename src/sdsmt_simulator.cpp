#include "sdsmt_simulator.h"

SDSMT_Simulator::SDSMT_Simulator(MapLoader_If *mapLoad, RobotLoader_If *robotLoad,
                                 Simulator_Physics_If *physics, Simulator_Ui_If *ui,
                                 QObject *parent) :
QObject(parent),
_mapLoader(mapLoad), _robotLoader(robotLoad),
_physicsEngine(physics), _userInterface(ui)
{
    _physicsThread = new QThread(this);
    _physicsEngine->moveToThread(_physicsThread);

    connect(_userInterface, &Simulator_Ui_If::userRemoveRobotFromSimulation, this, &SDSMT_Simulator::removeSimRobot);
    connect(this, &SDSMT_Simulator::robotRemoved, _userInterface, &Simulator_Ui_If::robotRemovedFromSimulation);
    connect(this, &SDSMT_Simulator::robotRemoved, _physicsEngine, &Simulator_Physics_If::removeRobot);

    connect(_userInterface, &Simulator_Ui_If::userAddRobotIntoSimulation, this, &SDSMT_Simulator::addSimRobotFromFile);
    connect(this, static_cast<void (SDSMT_Simulator::*)(Robot_Physics*)>(&SDSMT_Simulator::robotAdded), _physicsEngine, &Simulator_Physics_If::addRobot);
    connect(this, static_cast<void (SDSMT_Simulator::*)(Robot_Properties*)>(&SDSMT_Simulator::robotAdded), _userInterface, &Simulator_Ui_If::robotAddedToSimulation);

    connect(_userInterface, &Simulator_Ui_If::userSetMapInSimulation, this, &SDSMT_Simulator::setSimMapFromFile);
    connect(this, &SDSMT_Simulator::mapObjectsLoaded, _physicsEngine, &Simulator_Physics_If::newStaticShapes);

    connect(_userInterface, &Simulator_Ui_If::userStartPhysics, _physicsEngine, &Simulator_Physics_If::start);
    connect(_userInterface, &Simulator_Ui_If::userStopPhysics, _physicsEngine, &Simulator_Physics_If::stop);
    connect(_userInterface, &Simulator_Ui_If::userSetPhysicsTick, _physicsEngine, &Simulator_Physics_If::setTick);

    connect(this, &SDSMT_Simulator::errorMsg, _userInterface, &Simulator_Ui_If::errorMessage);
}

SDSMT_Simulator::~SDSMT_Simulator()
{
    //Stop physics engine
    _physicsThread->quit();
    _physicsThread->wait();

    //Destroy all remaining robots
    for(auto iter = _activeRobots.begin(); iter != _activeRobots.end(); iter++)
    {
        removeSimRobot(iter.key());
    }
}

void SDSMT_Simulator::start()
{
    _userInterface->showMainWindow();
    _physicsThread->start();
}

void SDSMT_Simulator::setSimMapFromFile(QString file)
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

void SDSMT_Simulator::addSimRobotFromFile(QString file)
{
    Robot* newBot;
    QString error = _robotLoader->loadRobotFile(file, newBot);
    if(!error.size())
    {
        //Put robot in its own thread
        QThread* rThread = new QThread(this);
        newBot->moveToThread(rThread);

        //Send out robot interfaces
        emit robotAdded(new Robot_Physics(newBot, _nextRobotId));
        emit robotAdded(new Robot_Properties(newBot, _nextRobotId));

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

void SDSMT_Simulator::removeSimRobot(robot_id rId)
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
