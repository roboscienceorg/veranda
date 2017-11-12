#ifndef SDSMT_SIMULATOR_H
#define SDSMT_SIMULATOR_H

#include "interfaces/map_loader_if.h"
#include "interfaces/robot_loader_if.h"
#include "interfaces/simulator_physics_if.h"
#include "interfaces/simulator_ui_if.h"
#include "interfaces/world_object_if.h"
#include "interfaces/simulator_visual_if.h"
#include "map.h"

#include <QObject>
#include <QMap>
#include <QThread>

#include <functional>

class SimulatorCore : public QObject
{
    Q_OBJECT

    Simulator_Physics_If* _physicsEngine;
    Simulator_Ui_If* _userInterface;

    object_id _nextObject = 1;
    QMap<object_id, WorldObject_If*> _worldObjects;

public:
    SimulatorCore(Simulator_Physics_If* physics, Simulator_Ui_If* ui,
                   QObject* parent = nullptr);
    ~SimulatorCore();

    void start();

signals:
    void objectRemoved(object_id rId);

    void objectAdded(WorldObjectPhysics_If* interface, object_id id);
    void objectAdded(WorldObjectProperties_If* interface, object_id id);

    void errorMsg(QString);

    void userStartPhysics();
    void userStopPhysics();
    void userSetPhysicsTick(double, double);

    void physicsStarted();
    void physicsStopped();
    void physicsTickSet(double, double);

public slots:
    void addSimObject(WorldObject_If* obj);
    void removeSimObject(object_id oId);
};

#endif // SDSMT_SIMULATOR_H
