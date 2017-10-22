#ifndef SDSMT_SIMULATOR_H
#define SDSMT_SIMULATOR_H

#include "interfaces/map_loader_if.h"
#include "interfaces/robot_interfaces.h"
#include "interfaces/robot_loader_if.h"
#include "interfaces/simulator_physics_if.h"
#include "interfaces/simulator_ui_if.h"
#include "interfaces/simulator_visual_if.h"

#include <QObject>
#include <QMap>
#include <QThread>

#include <functional>

class SimulatorCore : public QObject
{
    Q_OBJECT

    MapLoader_If* _mapLoader;
    RobotLoader_If* _robotLoader;

    QThread* _physicsThread;
    Simulator_Physics_If* _physicsEngine;
    Simulator_Ui_If* _userInterface;

    robot_id _nextRobotId = 0;
    QMap<robot_id, QThread*> _robotThreads;
    QMap<robot_id, Robot*> _activeRobots;

public:
    SimulatorCore(MapLoader_If* mapLoad, RobotLoader_If* robotLoad,
                    Simulator_Physics_If* physics, Simulator_Ui_If* ui,
                    QObject* parent = nullptr);
    ~SimulatorCore();

    void start();

signals:
    void robotRemoved(robot_id rId);

    void robotAdded(Robot_Physics* interface);
    void robotAdded(Robot_Properties* interface);

    void mapObjectsLoaded(QVector<b2Shape*> shapes);

    void errorMsg(QString);

public slots:
    void setSimMapFromFile(QString file);

    void addSimRobotFromFile(QString file);
    void removeSimRobot(robot_id rId);
};

#endif // SDSMT_SIMULATOR_H
