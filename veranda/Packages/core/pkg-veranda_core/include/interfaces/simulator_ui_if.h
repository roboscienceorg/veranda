//! \file
#pragma once

#include <functional>

#include <QObject>
#include <QVector>
#include <QWidget>
#include <QMainWindow>

#include <Box2D/Box2D.h>
#include <veranda_core/world_object.h>

#include "interfaces/world_object_wrappers.h"

/*!
 * \brief Inteface between the user and the simulation
 * The Simulator UI has two main responsibilities
 *  * Display the currently active simulation
 *  * Allow the user to add/remove simulated objects and modify their properties
 *
 * The simulator UI may also provide the user with some sort of virtual joystick
 * that can be used to drive robots; this is not a requirement; however, signals
 * exist that should be used if this is the case. Each joystick should be keyed to the
 * ROS 2 channel it publishes on, and handlers of the joystick will create, manage, and
 * publish on those channels
 */
class Simulator_Ui_If : public QMainWindow
{
    Q_OBJECT

public:
    /*!
     * \brief Constructs a new simulator UI
     * \param[in] parent QObject parent
     */
    Simulator_Ui_If(QWidget* parent = nullptr) : QMainWindow(parent){}

signals:
    /*!
     * \brief Request the physic tick to be changed
     * \param[in] rate_hz The requested new tick rate
     * \param[in] duration_s The requested new time simulated per tick
     */
    void userSetPhysicsTick(double rate_hz, double duration_s);

    /*!
     * \brief Request the physics rate multiplier to be changed
     * \param[in] mult The requested new multiplier
     */
    void userSetPhysicsTickMultiplier(double mult);

    //! Request the physics engine to stop ticking
    void userStopPhysics();

    //! Request the physics engine to start ticking
    void userStartPhysics();

    /*!
     * \brief Request that world objects be added to the simulation
     * The request specifies if the receiver should create clones of the
     * pointers or not. If clone is false, the receiver should take ownership of the
     * objects, and the UI interface should not retain the pointers.
     *
     * \todo Remove the clone parameter and use shared pointers instead to resolve concurrency issues
     *
     * \param[in] objects The objects to add to the simulation
     * \param[in] clone If true, the receiver will not take ownership of the objects added
     */
    void userAddWorldObjectsToSimulation(QVector<WorldObject*> objects, bool clone);

    /*!
     * \brief Request to remove objects from the simulation
     * When objects are added to the simulation, they are assigned object_ids; those
     * ids should be used here to specify which objects are to be removed
     * \param[in] ids List of object ids associated with the objects to be removed
     */
    void userRemoveWorldObjectsFromSimulation(QVector<object_id> ids);

    /*!
     * \brief The user moved a virtual joystick
     * \param x[in] The current location of the joystick on the x axis [-1, 1]
     * \param y[in] The current location of the joystick on the y axis [-1, 1]
     * \param z[in] The current location of the joystick on the z axis [-1, 1]
     * \param[in] channel The ROS 2 channel name to publish this joystick on
     */
    void joystickMoved(double x, double y, double z, QString channel);

    /*!
     * \brief The user pressed a button on a virtual joystick
     * \param[in] button The integer value of the button
     * \param[in] channel The ROS 2 channel name to publish this joystick on
     */
    void joystickButtonPress(int button, QString channel);

    /*!
     * \brief The user released a button on a virtual joystick
     * \param[in] button The integer value of the button
     * \param[in] channel The ROS 2 channel name to publish this joystick on
     */
    void joystickButtonRelease(int button, QString channel);

public slots:
    /*!
     * \brief Handler for world objects being added to the simulation
     * World objects are wrapped in the WorldObjectProperties type to prevent the UI
     * from accessing functionality it should not. Each world object is mapped to an object_id, which
     * should be used to refer to that object after this call. The WorldObjectProperties
     * references are valid until worldObjectsRemovedFromSimulation() is passed the associated
     * object_id
     *
     * During this handler, the UI should get all models from the added objects and begin displaying
     * them. The display routines should listen for the models' update signals and redraw the models
     * when they change or move
     *
     * \param[in] objects List of the new objects paired with their assigned ids
     */
    virtual void worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>> objects) = 0;

    /*!
     * \brief Handler for world objects being removed from the simulation
     * \param[in] oIds The list of object ids of the objects removed
     */
    virtual void worldObjectsRemovedFromSimulation(QVector<object_id> oIds) = 0;

    /*!
     * \brief Updates the Ui interface when the physics tick information changes
     * \param[in] rate_hz The current rate of physics updates
     * \param[in] duration_s The current time simulated per update
     */
    virtual void physicsTickChanged(double rate_hz, double duration_s) = 0;

    /*!
     * \brief Updates the UI interface when the physics tick rate multiplier changes
     * \param[in] mult The new physics tick rate multiplier
     */
    virtual void physicsTickMultiplierChanged(double mult) = 0;

    //! Slot when the physics engine stopped ticking
    virtual void physicsStopped() = 0;

    //! Slot when the physics engine started ticking
    virtual void physicsStarted() = 0;

    /*!
     * \brief Displays an error message to the user
     * \param[in] error The text to display
     */
    virtual void errorMessage(QString error) = 0;

    //! Slot to display the main UI window on startup
    virtual void showMainWindow() = 0;
};
