//! \file
#pragma once

#include "veranda_core/interfaces/simulator_physics_if.h"
#include "veranda_core/dllapi.h"
#include <QTimer>

#include <QObject>
#include <QMap>

/*!
 * \brief Default implementation of the Simulator_Physics_If interface
 * This type inherits Simulator_Physics_If and implements all
 * of its virtual stubs. The clock is implemented using a QTimer
 *
 * \todo Support gravity for side-view simulations instead of top-down
 */
class veranda_API BasicPhysics : public Simulator_Physics_If
{
    Q_OBJECT

    //! Holds stored information for a specific WorldObject
    struct objectWorldData{
        WorldObjectPhysics *obj; //! Pointer to the WorldObjectPhysics wrapper
    };

    //! The physics world
    b2World *world;

    //! Mapping of objects by their id
    QMap<object_id, objectWorldData> objects;

    //! QTimer for ticking the physics
    QTimer* tick = nullptr;

    //! Current target tick rate
    double tickRate;

    //! Current time stepped each tick
    double stepTime;

    //! Current tick rate multiplier
    double tickMult;

public:
    /*!
     * \brief Constructs the BasicPhysics type
     * \param[in] parent QObject parent of the Physics
     */
    BasicPhysics(QObject* parent = nullptr);

    /*!
     * \brief Getter to check if the physics are currently running
     * \return True if the physics tick timer is running
     */
    bool running(){return tick->isActive();}

public slots:
    //! Slot to start the tick timer
    virtual void start() override;

    //! Slot to stop the tick timer
    virtual void stop() override;

    //! Slot to remove all simulated objects
    virtual void clear() override;

    /*!
     * \brief Slot to set the tick rate and duration and updates the QTimer with the new rate
     * \param[in] rate_hz Rate to tick simulation at
     * \param[in] duration_s Amount of time to simulate with each tick
     */
    virtual void setTick(double rate_hz, double duration_s) override;

    /*!
     * \brief Slot to set the tick rate multiplier
     * The rate multiplier is applied to the tick rate (as the name implies). A multipler
     * of 2 will cause the simulation to progress twice as fast (provided the CPU can support it)
     * \param[in] mult New multipler
     */
    virtual void setTickMultiplier(double mult) override;

    /*!
     * \brief Slot to add new WorldObjects  to the simulation
     * WorldObjects are wrapped in the WorldObjectPhysics type. Each one is
     * associated with an object id that should be used to reference the object
     * after it's been added
     * \param[in] objs List if new objects to add and their ids
     */
    virtual void addWorldObjects(QVector<QPair<WorldObjectPhysics*, object_id>> objs) override;

    /*!
     * \brief Removes world objects from the simulation physics
     * \param[in] oIds List of ids of the objects to remove
     */
    virtual void removeWorldObjects(QVector<object_id> oIds) override;

    //! Ticks the physics world forward the amount of time currently set
    void step();
};
