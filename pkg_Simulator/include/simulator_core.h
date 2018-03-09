#ifndef SDSMT_SIMULATOR_H
#define SDSMT_SIMULATOR_H

#include "interfaces/simulator_physics_if.h"
#include "interfaces/simulator_ui_if.h"
#include "interfaces/simulator_visual_if.h"
#include "interfaces/world_object_wrappers.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"

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

    struct joymsg
    {
        typedef sensor_msgs::msg::Joy msgType;

        std::shared_ptr<msgType> _message = nullptr;
        std::shared_ptr<rclcpp::Publisher<msgType>> _channel = nullptr;
    };

    QMap<QString, joymsg> _joysticks;

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> _timestampChannel;
    std::shared_ptr<std_msgs::msg::Float64MultiArray> _timestampMsg;

public:
    SimulatorCore(Simulator_Physics_If* physics, Simulator_Ui_If* ui, std::shared_ptr<rclcpp::Node> node,
                   QObject* parent = nullptr);
    ~SimulatorCore();

    void start();

private:
    joymsg initJoystick(QString channel);

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

private slots:
    void joystickMoved(double x, double y, double z, QString channel);
    void joystickButtonDown(int button, QString channel);
    void joystickButtonUp(int button, QString channel);

    void clearJoystickChannels();

public slots:
    void addSimObjects(QVector<QSharedPointer<WorldObject> > objs);
    void removeSimObjects(QVector<object_id> oIds);
};

#endif // SDSMT_SIMULATOR_H
