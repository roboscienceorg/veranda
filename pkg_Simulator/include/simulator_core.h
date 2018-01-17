#ifndef SDSMT_SIMULATOR_H
#define SDSMT_SIMULATOR_H

#include "interfaces/simulator_physics_if.h"
#include "interfaces/simulator_ui_if.h"
#include "sdsmt_simulator/world_object.h"
#include "interfaces/simulator_visual_if.h"
#include "map.h"

#include "rclcpp/rclcpp.hpp"

#include <QObject>
#include <QMap>
#include <QThread>

#include <memory>
#include <functional>

class SimulatorCore : public QObject
{
    Q_OBJECT

    Simulator_Physics_If* _physicsEngine;
    Simulator_Ui_If* _userInterface;

    object_id _nextObject = 1;
    QMap<object_id, WorldObject*> _worldObjects;

    std::shared_ptr<rclcpp::Node> _node;

public:
    SimulatorCore(Simulator_Physics_If* physics, Simulator_Ui_If* ui, std::shared_ptr<rclcpp::Node> node,
                   QObject* parent = nullptr);
    ~SimulatorCore();

    void start();

signals:
    void objectRemoved(object_id rId);

    void objectAdded(WorldObjectPhysics* interface, object_id id);
    void objectAdded(WorldObjectProperties* interface, object_id id);

    void errorMsg(QString);

    void userStartPhysics();
    void userStopPhysics();
    void userSetPhysicsTick(double, double);

    void physicsStarted();
    void physicsStopped();
    void physicsTickSet(double, double);

public slots:
    void addSimObject(WorldObject* obj);
    void removeSimObject(object_id oId);
};

#endif // SDSMT_SIMULATOR_H
