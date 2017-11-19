#include "simulator_core.h"

#include <iostream>
#include <QDebug>
#include <QTimer>
#include <QThread>

using namespace std;

SimulatorCore::SimulatorCore(Simulator_Physics_If *physics, Simulator_Ui_If *ui,
                                 QObject *parent) :
QObject(parent),
_physicsEngine(physics), _userInterface(ui)
{
    qRegisterMetaType<object_id>("object_id");

    connect(_userInterface, &Simulator_Ui_If::userRemoveWorldObjectFromSimulation, this, &SimulatorCore::removeSimObject);
    connect(this, &SimulatorCore::objectRemoved, _userInterface, &Simulator_Ui_If::worldObjectRemovedFromSimulation);
    connect(this, &SimulatorCore::objectRemoved, _physicsEngine, &Simulator_Physics_If::removeWorldObject);

    connect(_userInterface, &Simulator_Ui_If::userAddWorldObjectToSimulation, this, &SimulatorCore::addSimObject);
    //connect(this, static_cast<void (SimulatorCore::*)(WorldObjectPhysics*, object_id)>(&SimulatorCore::objectAdded),
    //        _physicsEngine, &Simulator_Physics_If::addWorldObject);
    connect(this, static_cast<void (SimulatorCore::*)(WorldObjectProperties*, object_id)>(&SimulatorCore::objectAdded),
            _userInterface, &Simulator_Ui_If::worldObjectAddedToSimulation);

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
    //Destroy all remaining objects
    while(_worldObjects.size())
        removeSimObject(_worldObjects.firstKey());

    //Stop physics engine
    userStopPhysics();
    _physicsEngine->deleteLater();

    //Destroy ui
    _userInterface->deleteLater();
}

void SimulatorCore::start()
{
    //Set default physics tick rate
    userSetPhysicsTick(100.0, 1.0/100.0);

    //Show UI
    _userInterface->showMainWindow();
}


void SimulatorCore::addSimObject(WorldObject *obj)
{
    //Clone object to have local copy for distributing
    obj = obj->clone();

    connect(_physicsEngine, &Simulator_Physics_If::physicsStarted, obj, &WorldObject::connectChannels);
    connect(_physicsEngine, &Simulator_Physics_If::physicsStopped, obj, &WorldObject::disconnectChannels);

    //WorldObjectPhysics* phys_interface = new WorldObjectPhysics(obj, obj);
    WorldObjectProperties* prop_interface = new WorldObjectProperties(obj, obj);

    //Send out object interfaces
    //emit objectAdded(phys_interface, _nextObject);
    emit objectAdded(prop_interface, _nextObject);

    //Keep references to robot and thread
    _worldObjects[_nextObject] = obj;

    _nextObject++;

    if(_physicsEngine->running())
        obj->connectChannels();
    else
        obj->disconnectChannels();

    /*Map* asMap = qobject_cast<Map*>(obj);
    if(asMap)
    {
        double xMin, xMax, yMin, yMax;
        asMap->getBounds(xMin, yMin, xMax, yMax);
        _userInterface->setWorldBounds(xMin, xMax, yMin, yMax);
    }*/
}

void SimulatorCore::removeSimObject(object_id oId)
{
    if(_worldObjects.contains(oId))
    {
        //Signal that object is no longer in simulation
        emit objectRemoved(oId);

        //Delete object
        //This should delete all object interfaces; they are children to it
        delete _worldObjects[oId];
        _worldObjects.remove(oId);
    }
}
