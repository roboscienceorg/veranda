//! \file
#include "simulator_core.h"

#include <iostream>
#include <QDebug>
#include <QTimer>
#include <QThread>
#include <QSharedPointer>

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

    //Every time the user stops or starts the physics simulation, we'll clear out our cache of
    //joystick channels; this should prevent it from getting too big ever and hogging memory
    connect(this, &SimulatorCore::physicsStopped, this, &SimulatorCore::clearJoystickChannels);
    connect(this, &SimulatorCore::physicsStarted, this, &SimulatorCore::clearJoystickChannels);

    connect(_userInterface, &Simulator_Ui_If::userStartPhysics, this, &SimulatorCore::userStartPhysics);
    connect(_userInterface, &Simulator_Ui_If::userStopPhysics, this, &SimulatorCore::userStopPhysics);
    connect(_userInterface, &Simulator_Ui_If::userSetPhysicsTick, this, &SimulatorCore::userSetPhysicsTick);
    connect(_userInterface, &Simulator_Ui_If::userSetPhysicsTickMultiplier, _physicsEngine, &Simulator_Physics_If::setTickMultiplier);

    connect(_physicsEngine, &Simulator_Physics_If::physicsStarted, this, &SimulatorCore::physicsStarted);
    connect(_physicsEngine, &Simulator_Physics_If::physicsStopped, this, &SimulatorCore::physicsStopped);
    connect(_physicsEngine, &Simulator_Physics_If::physicsTickSet, this, &SimulatorCore::physicsTickSet);
    connect(_physicsEngine, &Simulator_Physics_If::physicsTickMultiplierSet, _userInterface, &Simulator_Ui_If::physicsTickMultiplierChanged);

    connect(_userInterface, &Simulator_Ui_If::joystickButtonPress, this, &SimulatorCore::joystickButtonDown);
    connect(_userInterface, &Simulator_Ui_If::joystickButtonRelease, this, &SimulatorCore::joystickButtonUp);
    connect(_userInterface, &Simulator_Ui_If::joystickMoved, this, &SimulatorCore::joystickMoved);

    connect(this, &SimulatorCore::errorMsg, _userInterface, &Simulator_Ui_If::errorMessage);

    //Create and set up timestamp message
    _timestampMsg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    _timestampMsg->data.resize(2, 0);
    _timestampMsg->layout.data_offset = 0;
    _timestampMsg->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    _timestampMsg->layout.dim[0].label = "times";
    _timestampMsg->layout.dim[0].size = 2;
    _timestampMsg->layout.dim[0].stride = 2;

    //Create channel to publish timestamps on
    _timestampChannel = _node->create_publisher<std_msgs::msg::Float64MultiArray>("veranda_core/timestamp", 10);

    //Publish on every physics tick
    connect(_physicsEngine, &Simulator_Physics_If::physicsTicked,
    [this](double elapsed)
    {
        _timestampMsg->data[0] += elapsed;
        _timestampMsg->data[1] = elapsed;
        _timestampChannel->publish(_timestampMsg);
    });
}

/*!
 * When it is destroyed, the simulator core will destroy all
 * of the objects in the simulation, and then delete the physics
 * and UI interfaces
 */
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

/*!
 * On startup, the physics ticks are set to 1/30, 30 times a second
 */
void SimulatorCore::start()
{
    //Set default physics tick rate
    _physicsEngine->setTick(30.0, 1.0/30.0);
    _physicsEngine->setTickMultiplier(1.0);

    //Show UI
    _userInterface->showMainWindow();
}


void SimulatorCore::addSimObjects(QVector<WorldObject*> objs, bool makeCopies)
{
    qDebug() << "Loading batch of" << objs.size() << "Objects";
    QVector<QPair<WorldObjectPhysics*, object_id>> physObjs;
    QVector<QPair<WorldObjectProperties*, object_id>> propObjs;

    for(WorldObject* oldObj : objs)
    {
        //Clone object to have local copy for distributing
        WorldObject* obj = qobject_cast<WorldObject*>(oldObj->clone());
        if(!makeCopies) oldObj->deleteLater();

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
    qDebug() << "Added to physics";
    emit objectsAdded(propObjs);
    qDebug() << "Added to view";
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

void SimulatorCore::joystickMoved(double x, double y, double z, QString channel)
{
    if(!channel.size()) return;
    joymsg joystick = initJoystick(channel);

    joystick._message->axes.resize(3);
    joystick._message->axes[0] = x;
    joystick._message->axes[1] = y;
    joystick._message->axes[2] = z;

    if(joystick._channel)
    {
        joystick._channel->publish(joystick._message);
    }
}

void SimulatorCore::joystickButtonDown(int button, QString channel)
{
    if(!channel.size() || button > 256) return;

    joymsg joystick = initJoystick(channel);

    if(joystick._message->buttons.size() <= button)
        joystick._message->buttons.resize(button+1);

    joystick._message->buttons[button] = 1;

    if(joystick._channel)
    {
        joystick._channel->publish(joystick._message);
    }
}

void SimulatorCore::joystickButtonUp(int button, QString channel)
{
    if(!channel.size() || button > 256) return;

    joymsg joystick = initJoystick(channel);

    if(joystick._message->buttons.size() <= button)
        joystick._message->buttons.resize(button+1);

    joystick._message->buttons[button] = 0;

    if(joystick._channel)
    {
        joystick._channel->publish(joystick._message);
    }
}

void SimulatorCore::clearJoystickChannels()
{
    _joysticks.clear();
}

SimulatorCore::joymsg SimulatorCore::initJoystick(QString channel)
{
    joymsg& joy = _joysticks[channel];
    if(!joy._channel)
    {
        if(_node)
        {
            joy._channel = _node->create_publisher<joymsg::msgType>(channel.toStdString(), 7);
        }
    }

    if(!joy._message)
    {
        joy._message = make_shared<joymsg::msgType>();
    }

    return joy;
}
