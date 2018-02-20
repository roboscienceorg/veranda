#ifndef SDSMT_SIMULATOR_H
#define SDSMT_SIMULATOR_H

#include "interfaces/simulator_physics_if.h"
#include "interfaces/simulator_ui_if.h"
#include "interfaces/simulator_visual_if.h"
#include "interfaces/world_object_wrappers.h"

#include "rclcpp/rclcpp.hpp"
#include <sdsmt_simulator/world_object.h>

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
    void objectsRemoved(QVector<object_id> rId);

    void objectsAdded(QVector<QPair<WorldObjectPhysics*, object_id>> objs);
    void objectsAdded(QVector<QPair<WorldObjectProperties*, object_id>> objs);

    void errorMsg(QString);

    void userStartPhysics();
    void userStopPhysics();
    void userSetPhysicsTick(double, double);

    void physicsStarted();
    void physicsStopped();
    void physicsTickSet(double, double);

public slots:
    void addSimObjects(QVector<WorldObject*> objs);
    void removeSimObjects(QVector<object_id> oIds);
};

#endif // SDSMT_SIMULATOR_H
