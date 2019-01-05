//! \file
#pragma once

#include <QObject>
#include <QVector>

#include <Box2D/Box2D.h>

#include "veranda_core/interfaces/world_object_wrappers.h"
#include "veranda_core/dllapi.h"

/*!
 * \brief Interface between the physics engine and the rest of the simulation
 * The Simulator Physics If is responsible for moving the simulation
 * forward at a regular rate and keeping track of what objects have
 * had their bodies added to it. It also needs to be able to vary the
 * rate of simulation to be able to optimize for a specific computer
 * and provide the time warping functionalities
 */
class veranda_API Simulator_Physics_If : public QObject
{
    Q_OBJECT

public:
    /*!
     * \brief Constructs a new Simulation Physics
     * \param[in] parent QObject parent
     */
    Simulator_Physics_If(QObject* parent = nullptr) : QObject(parent){}

    /*!
     * \brief Getter to check if the simulation clock is currently active
     * \return True if the simulation clock is running
     */
    virtual bool running() = 0;

public slots:
    //! Starts the simulaiton clock
    virtual void start() = 0;

    //! Stops the simulation clock
    virtual void stop() = 0;

    //! Removes all objects from the physics engine
    virtual void clear() = 0;

    /*!
     * \brief Sets the rate of physics engine ticks, and the length simulated in each
     * \param[in] rate_hz Rate at which the simulation should update
     * \param[in] duration_s Amount of time (seconds) to simulate on each update
     */
    virtual void setTick(double rate_hz, double duration_s) = 0;

    /*!
     * \brief Sets a scaling factor for the simulation
     * The rate of updates is multiplied by this multiplier, so to
     * run the simulation twice as fast, set the tick multiplier to be 2.0
     * \param[in] mult Multiplier for tick rate
     */
    virtual void setTickMultiplier(double mult) = 0;

    /*!
     * \brief Registers new WorldObjects with the physics engine
     * World Objects are wrapped in the WorldObjectPhysics interface to prevent
     * the physics engine accessing functionality it should not, and each
     * World Object is paired with an object id which is used to reference it at
     * future times
     *
     * \param[in] objects The list of new WorldObjects
     */
    virtual void addWorldObjects(QVector<QPair<WorldObjectPhysics*, object_id>> objects) = 0;

    /*!
     * \brief Removes 0 or more world objects from the physics engine
     * This calling this method should essentially undo addWorldObjects for all
     * world objects that were paired with the object_ids given in this call. Those
     * objects should be removed from the simulation and their references dropped to prevent
     * dangling pointers
     *
     * \param[in] objects The object_ids of the objects to remove from the physics engine
     */
    virtual void removeWorldObjects(QVector<object_id> objects) = 0;

signals:
    //! Signals that the physics clock has started
    void physicsStarted();

    //! Signals that the physics clock has stopped
    void physicsStopped();

    /*!
     * \brief Signal that the tick rate and/or time were set
     * This signal is not emitted when the physics tick multiplier
     * is changed
     * \param[in] rate_hz Rate of physics ticking
     * \param[in] time_s Amount of time simulated on each tick
     */
    void physicsTickSet(double rate_hz, double time_s);

    /*!
     * \brief Signal that the multiplier value on the physics tick was set
     * \param[in] mult The multiplier on the tick rate
     */
    void physicsTickMultiplierSet(double mult);

    /*!
     * \brief Signal that the physics engine simulated some amount of time
     * This signal will be emitted on a high-frequency timer, so do
     * minimal processing in slots connected to it
     * \param[in] seconds The amount of time that was simulated
     */
    void physicsTicked(double seconds);
};
