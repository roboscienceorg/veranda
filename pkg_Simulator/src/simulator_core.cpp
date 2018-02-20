#include "simulator_core.h"

#include <iostream>
#include <QDebug>
#include <QTimer>
#include <QThread>

using namespace std;

SimulatorCore::SimulatorCore(Simulator_Physics_If *physics, Simulator_Ui_If *ui, std::shared_ptr<rclcpp::Node> node,
                                 QObject *parent) :
QObject(parent),
_physicsEngine(physics), _userInterface(ui), _node(node)
{
    qRegisterMetaType<object_id>("object_id");

    connect(_userInterface, &Simulator_Ui_If::userRemoveWorldObjectsFromSimulation, this, &SimulatorCore::removeSimObjects);
    connect(this, &SimulatorCore::objectsRemoved, _userInterface, &Simulator_Ui_If::worldObjectsRemovedFromSimulation);
    connect(this, &SimulatorCore::objectsRemoved, _physicsEngine, &Simulator_Physics_If::removeWorldObjects);

    connect(_userInterface, &Simulator_Ui_If::userAddWorldObjectsToSimulation, this, &SimulatorCore::addSimObjects);
    connect(this, static_cast<void (SimulatorCore::*)(QVector<QPair<WorldObjectPhysics*, object_id>>)>(&SimulatorCore::objectsAdded),
            _physicsEngine, &Simulator_Physics_If::addWorldObjects);
    connect(this, static_cast<void (SimulatorCore::*)(QVector<QPair<WorldObjectProperties*, object_id>>)>(&SimulatorCore::objectsAdded),
            _userInterface, &Simulator_Ui_If::worldObjectsAddedToSimulation);

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
    removeSimObjects(_worldObjects.keys().toVector());

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


void SimulatorCore::addSimObjects(QVector<WorldObject *> objs)
{
    QVector<QPair<WorldObjectPhysics*, object_id>> physObjs;
    QVector<QPair<WorldObjectProperties*, object_id>> propObjs;

    for(WorldObject* obj : objs)
    {
        //Clone object to have local copy for distributing
        obj = obj->clone();

        obj->setROSNode(_node);

        connect(_physicsEngine, &Simulator_Physics_If::physicsStarted, obj, &WorldObject::connectChannels);
        connect(_physicsEngine, &Simulator_Physics_If::physicsStopped, obj, &WorldObject::disconnectChannels);

        physObjs += {new WorldObjectPhysics(obj, obj), _nextObject};
        propObjs += {new WorldObjectProperties(obj, obj), _nextObject};

        //Keep references to robot and thread
        _worldObjects[_nextObject] = obj;

        _nextObject++;

        if(_physicsEngine->running())
            obj->connectChannels();
        else
            obj->disconnectChannels();
    }

    //Send out object interfaces
    emit objectsAdded(physObjs);
    emit objectsAdded(propObjs);

}

void SimulatorCore::removeSimObjects(QVector<object_id> oIds)
{
    QVector<object_id> removes;
    for(object_id oId : oIds)
        if(_worldObjects.contains(oId))
            removes += oId;

    //Signal that object is no longer in simulation
    emit objectsRemoved(removes);

    for(object_id oId : removes)
    {
        //Delete object
        //This should delete all object interfaces; they are children to it
        delete _worldObjects[oId];
        _worldObjects.remove(oId);
    }
}
