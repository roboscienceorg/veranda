//! \file
#pragma once

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

#include <veranda/world_object.h>

#include <QObject>
#include <QMap>
#include <QThread>

#include <memory>
#include <functional>

/*!
 * \brief As the name implies, this is the central piece of the simulation. It organzies the physics, ui, and world objects
 * The simulator core is mainly a mediator between the UI and Physics portions of the simulation. It creates and holds the original
 * references to each World Object and sends each other part its reference wrapper.
 *
 * The simulator core is also responsible for publishing joystick and timestamp messages. Joystick messages are driven
 * by the UI portion of the application, and the timestamps are driven by the physics. Joystick information accumulates over
 * runtime whenever a new joystick channel is requested. This information is cleared when the simulation stops or starts
 * to prevent it getting too big
 */
class SimulatorCore : public QObject
{
    Q_OBJECT

    //! The physics interface for the simulation
    Simulator_Physics_If* _physicsEngine;

    //! The user interface for the simulation
    Simulator_Ui_If* _userInterface;

    //! Counter for assigning object ids. This begins at 1, so id 0 can safely be used as a non-object anywhere
    object_id _nextObject = 1;

    //! Map of object_id to world objects in simulation
    QMap<object_id, WorldObject*> _worldObjects;

    //! The ROS 2 node for the application
    std::shared_ptr<rclcpp::Node> _node;

    //! Container for information related to a single joystick's data
    struct joymsg
    {
        //! Typedef to shorten joystick message type
        typedef sensor_msgs::msg::Joy msgType;

        //! The joystick message being sent
        std::shared_ptr<msgType> _message = nullptr;

        //! The channel this joystick message is sent on
        std::shared_ptr<rclcpp::Publisher<msgType>> _channel = nullptr;
    };

    //! Cache of joystick messages
    QMap<QString, joymsg> _joysticks;

    //! The ROS 2 channel to publish timestamp messages on
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> _timestampChannel;

    //! The most recently published timestamp message
    std::shared_ptr<std_msgs::msg::Float64MultiArray> _timestampMsg;

public:
    /*!
     * \brief Constructs a new simulator core. The core is built from a physics interface, a ui interface, and a ROS 2 node
     * \param[in] physics The physics interface
     * \param[in] ui The UI interface
     * \param[in] node The ROS 2 Node for the process
     * \param[in] parent The QObject parent
     */
    SimulatorCore(Simulator_Physics_If* physics, Simulator_Ui_If* ui, std::shared_ptr<rclcpp::Node> node,
                   QObject* parent = nullptr);
    ~SimulatorCore();

    /*!
     * \brief Activates the simulation after construction
     */
    void start();

private:
    /*!
     * \brief Checks if joystick information is known for a channel, and sets it up if not
     * \param[in] channel The topic for the joystick
     * \return The joymsg data for the joystick channel
     */
    joymsg initJoystick(QString channel);

signals:
    /*!
     * \brief Signal that objects are being removed from the simulation
     * \param[in] rId A list of object ids to remove
     */
    void objectsRemoved(QVector<object_id> rId);

    /*!
     * \brief Signal to the Physics that Objects are added to the simulation
     * \param[in] objs List of objects wrapped for the Physics and their object ids
     */
    void objectsAdded(QVector<QPair<WorldObjectPhysics*, object_id>> objs);

    /*!
     * \brief Signal to the UI that Objects are added to the simulation
     * \param[in] objs List of objects wrapped for the UI and their object ids
     */
    void objectsAdded(QVector<QPair<WorldObjectProperties*, object_id>> objs);

    /*!
     * \brief Signal an error message to be displayed
     */
    void errorMsg(QString);

    /*!
     * \brief Forwarding signal that user wants to start the physics clock
     */
    void userStartPhysics();

    /*!
     * \brief Forwarding signal that user wants to stop the physics clock
     */
    void userStopPhysics();

    /*!
     * \brief Forwarding signal that user wants to set the physics tick
     */
    void userSetPhysicsTick(double, double);

    /*!
     * \brief Forwarding signal that the physics started
     */
    void physicsStarted();

    /*!
     * \brief Forwarding signal the physics stopped
     */
    void physicsStopped();

    /*!
     * \brief Forwarding signal that physics ticks were set
     */
    void physicsTickSet(double, double);

private slots:
    /*!
     * \brief Publish joystick information when a virtual joystick moves
     * \param[in] x The joystick location on x axis
     * \param[in] y The joystick location on y axis
     * \param[in] z The joystick location on z axis
     * \param[in] channel The joystick topic to publish on
     */
    void joystickMoved(double x, double y, double z, QString channel);

    /*!
     * \brief Publish joystick information when a virtual button is pressed
     * \param[in] button The button number pressed
     * \param[in] channel The joystick topic to publish on
     */
    void joystickButtonDown(int button, QString channel);

    /*!
     * \brief Publish joystick information when a virtual button is released
     * \param[in] button The button number released
     * \param[in] channel The joystick topic to publish on
     */
    void joystickButtonUp(int button, QString channel);

    /*!
     * \brief Clears all cached info about joysticks. It will be recreated when joystick messages are published again.
     */
    void clearJoystickChannels();

public slots:
    /*!
     * \brief Add objects to the simulation
     * \param[in] objs The objects to add
     * \param[in] makeCopies Whether or not the simulator core should duplicate the objects
     */
    void addSimObjects(QVector<WorldObject *> objs, bool makeCopies);

    /*!
     * \brief Remove objects from the simulation
     * \param[in] oIds The object_ids of the objects to remove
     */
    void removeSimObjects(QVector<object_id> oIds);
};
