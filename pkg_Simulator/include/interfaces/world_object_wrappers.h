//! \file
//! \todo Maybe just remove these wrappers completely?
#pragma once

#include <sdsmt_simulator/world_object.h>
#include <sdsmt_simulator/property.h>
#include <sdsmt_simulator/model.h>
#include <Box2D/Box2D.h>

#include <QObject>
#include <QMap>


/*!
 * \brief Wrapper to filter UI access to WorldObjectComponents
 * Since QObject does not allow virtual inheritance to
 * solve the diamond problem, this wrapper exists so that the
 * UI does not have access to the WorldObjectComponent methods used
 * in the physics engine
 */
class WorldObjectProperties : public QObject
{
    Q_OBJECT

    //! The world object component observed by this wrapper
    WorldObjectComponent* _comp = nullptr;

public:
    /*!
     * \brief Constructs a WorldObjectProperties to observe a specific WorldObject
     * \param[in] comp The WorldObject to provide access to
     * \param[in] parent QObject parent of the wrapper
     */
    WorldObjectProperties(WorldObjectComponent* comp, QObject* parent=nullptr) : QObject(parent), _comp(comp){}

    /*!
     * See WorldObjectComponent::getModels
     * \return The models of the observed component
     */
    QVector<Model*> getModels()
    { return _comp->getModels(); }

    /*!
     * See WorldObjectComponent::getName()
     * \return The name of the observed component
     */
    QString getName()
    { return _comp->getName(); }

    /*!
     * See WorldObjectComponent::getName()
     * \return The type of the observed component
     */
    QString getType()
    { return _comp->getType(); }

    /*!
     * See WorldObjectComponent::getProperties()
     * \return The properties of the observed component
     */
    QMap<QString, QSharedPointer<PropertyView>> getProperties()
    { return _comp->getProperties(); }

    /*!
     * See WorldObjectComponent::usesChannels()
     * \return Whether or not the observed component uses channels
     */
    bool usesChannels()
    { return _comp->usesChannels(); }

    /*!
     * \brief Casts the observed WorldObjectComponent to a WorldObject
     * Yes, this really defeats the purpose of the wrapper; but it was necessary
     * for the UI to be able to get the full WorldObject type for composite Components
     * so that they could be saved in files
     * \return The internal WorldObjectComponent cast to a WorldObject
     */
    WorldObject* getObject()
    { return dynamic_cast<WorldObject*>(_comp); }

    /*!
     * \brief Getter for the observed component
     * Like getObject(), this method defeats the purpose of the wrapper, but was
     * necessary for saving and loading in files
     * \return The observed component
     */
    WorldObjectComponent* getComponent()
    { return _comp; }

    /*!
     * See WorldObjectComponent::translate()
     * \param[in] x Amount of horizontal translation
     * \param[in] y Amount of vertical translation
     */
    void translate(double x, double y){ _comp->translate(x, y); }

    /*!
     * See WorldObjectComponent::rotate()
     * \param[in] degrees Number of degrees of rotation
     */
    void rotate(double degrees){ _comp->rotate(degrees); }

public slots:
    //! See WorldObjectComponent::connectChannels()
    void connectChannels()
    { _comp->connectChannels(); }

    //! See WorldObjectComponent::disconnectChannels()
    void disconnectChannels()
    { _comp->disconnectChannels(); }
};

/*!
 * \brief Wrapper to filter UI access to WorldObjectComponents
 * Since QObject does not allow virtual inheritance to
 * solve the diamond problem, this wrapper exists so that the
 * physics engine does not have access to the WorldObjectComponent methods used
 * in the UI
 */
class WorldObjectPhysics : public QObject
{
    Q_OBJECT

    //! The wrapped world object
    WorldObject* _obj;

public:
    /*!
     * \brief Constructs a WorldObjectPhysics to observe a specific WorldObject
     * \param[in] obj The WorldObject to provide access to
     * \param[in] parent QObject parent of the wrapper
     */
    WorldObjectPhysics(WorldObject* obj, QObject* parent=nullptr) : QObject(parent), _obj(obj){}

    /*!
     * See WorldObjectComponent::generateBodies
     * \param[in] world b2World to add the world object to
     * \param[in] oId Object id of the object added
     */
    virtual void generateBodies(b2World* world, object_id oId){_obj->generateBodies(world, oId);}

    //! See WorldObjectComponent::clearBodies()
    virtual void clearBodies(){_obj->clearBodies();}

public slots:
    //! See WorldObjectComponent::syncModels()
    virtual void syncModels(){_obj->syncModels();}

    /*!
     * See WorldObjectComponent::worldTicked()
     * \param[in] t Amount of time passed in this tick
     */
    virtual void worldTicked(const double t){_obj->worldTicked(t);}
};
