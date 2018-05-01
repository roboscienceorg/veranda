//! \file
#pragma once

#include <QObject>
#include <QString>
#include <QMap>
#include <QTransform>
#include <QSharedPointer>

#include <memory>

#include "const.h"
#include "rclcpp/rclcpp.hpp"
#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/property.h"
#include "dllapi.h"

/*!
 * \brief Component type used to create world objects for simulations
 * The WorldObjectComponent is the main datatype of the project. Almost
 * all WorldObjectComponent types are defined in plugins that are loaded on startup.
 * A WorldObjectComponent can be described as a set of Properties and Models and rules
 * to build Box2D bodies for them and define how they behave in a simulation.
 *
 * Every WorldObjectComponent has a coordinate location x, y, theta as well as a name. The
 * location information is used to position the models and bodies of the components in
 * the global space, regardless of if the component is originally defined relative to other
 * components. These values are also used to automatically move and redraw components when
 * they are moved around in the simulation/worldobject editor
 *
 * Both bodies and models can be registered to a component. Registering models
 * and bodies will ensure that they move correctly when position properties are
 * modified, and keep the models in sync with the physics bodies. All registered
 * models and bodies should be unregistered before they are deleted. Only top-level
 * models should be registered; models that are children of registered models should not
 * be registered.
 *
 * During simulation, the locations of bodies takes precedence. Any models tied
 * to bodies will move with them, and any model not tied to bodies will not move.
 * If no bodies are registered (i.e. the component is not in simulation), then
 * models will move as a group according to the properties of the component.
 *
 * Components should subscribe to the requestedValue() signal of their properties
 * and rebuild bodies and/or models when they change. This will result in a responsive
 * user experience in the designer and allow users to see any visual effects of
 * changing property values.
 */
class SDSMT_SIMULATOR_API WorldObjectComponent : public QObject
{
    Q_OBJECT

    //! Property for component name
    Property _objName;

    //! Property for local x location
    Property _locX = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "X coord of the object relative to its parent"),
                                QVariant(0.0), &Property::double_validator);

    //! Property for local y location
    Property _locY = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Y coord of the object relative to its parent"),
                                QVariant(0.0), &Property::double_validator);

    //! Property for local rotation
    Property _locTheta = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Angle of the object (Degrees) relative to its parent"),
                                QVariant(0.0), &Property::angle_validator);

    //! Property for global x location
    Property _globX = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Global X coord of the object"),
                                QVariant(0.0), &Property::double_validator);

    //! Property for global y location
    Property _globY = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Global Y coord of the object"),
                                QVariant(0.0), &Property::double_validator);

    //! Property for global rotation
    Property _globTheta = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Global Angle of the object (Degrees)"),
                                QVariant(0.0), &Property::angle_validator);

    //! Mapping of all properties and their identifying keys
    QMap<QString, Property*> _properties
    {
        {"Name", &_objName},
        {"LocalPos/X", &_locX},
        {"LocalPos/Y", &_locY},
        {"LocalPos/Theta", &_locTheta},
        {"GlobalPos/X", &_globX},
        {"GlobalPos/Y", &_globY},
        {"GlobalPos/Theta", &_globTheta}
    };

    //! The main body of the component
    b2Body* _mainBody = nullptr;

    //! List of registered models
    QVector<Model*> _models;

    //! List of registered bodies and the models tied to them
    QMap<b2Body*, QVector<Model*>> _bodies;

    //! Transform matrix of the parent to this component
    QTransform parentTransform;

    //! Inverse of parentTransform cached for quick access
    QTransform parentInverse;

    //! Local transformation matrix
    QTransform localTransform;

    //! Global transformation matrix. = localTransform * parentTransform
    QTransform globalTransform;

    //! Global angle in radians
    double globalRadians = 0;

    //! Global location x, y
    b2Vec2 globalPos = b2Vec2(0, 0);

    //! Whether or not any parent transform has been set
    bool hasParent = false;

    //! Type identifier for the component
    QString _type = "";

    //! Default name for the component
    QString _defaultName = "";

    /*!
     * \brief Moves the component to a new location and updates all models and bodies
     * Models and bodies moved are kept in the same location relative to the origin of the component
     * \param[in] tOldI Inverse of the old transform
     * \param[in] tNew The new transform
     */
    void shiftComponent(const QTransform& tOldI, const QTransform& tNew);

    /*!
     * \brief Updates the location property values of the component to match its transforms
     */
    void updateProperties();

protected:
    /*!
     * \brief Constrains a body's location relative to the component origin
     * \param[in] bod The body to register and move
     * \param[in] representations 0 or more Models to tie to the location of the body
     * \param[in] isMainBody Whether or not the body should registered as the main one for the component
     */
    void registerBody(b2Body* bod, const QVector<Model*>& representations = {}, bool isMainBody = false);

    /*!
     * \brief Removes the constraint between a body and the component
     * \param[in] bod The body to unregister
     */
    void unregisterBody(b2Body* bod);

    /*!
     * \brief Constrains a models's location relative to the component origin
     * \param[in] mod The model to register and move
     */
    void registerModel(Model* mod);

    /*!
     * \brief Removes the constraint between a model and the component
     * \param[in] mod The model to unregister
     */
    void unregisterModel(Model* mod);

    /*!
     * \brief Gets the properties specific to inheriting types
     * \return A string mapping of PropertyView pointers
     */
    virtual QMap<QString, QSharedPointer<PropertyView>> _getProperties(){return {};}

    /*!
     * \brief Slot to update when the physics ticks in inheriting types
     * The physics engine will move the simulation forward to simulate
     * some delta time and then this slot will trigger. All time calculations
     * should be done using the delta times from the simulator and not using
     * system clock or wall time. This method should be used to read state
     * from the physics engine and act on it, either by changing internal
     * properties, publishing messages, or applying forces to physics bodies.
     *
     * \param[in] dt The amount of time simulated by the physics engine
     */
    virtual void _worldTicked(const double dt){}

    /*!
     * \brief Slot indicating that inheriting types should update their models
     * Any models not tied to bodies via the registerBody() function will not move
     * during simulation. During this slot, inheriting types should examine state
     * and update such models accordingly so that they move if desired.
     */
    virtual void _syncModels(){}

    /*!
     * \brief Virtual clone/copy
     * The _clone function is used to create a duplicate of the component.
     * _clone() should create an initialize a new component of the same type
     * as this one, and copy all data important to the class other than the
     * properties. Properties are copied by the WorldObjectComponent parent
     * class (see clone())
     *
     * \param[in] newParent QObject parent of the new object
     * \return A newly constructed WorldObjectComponent* of the same type
     */
    virtual WorldObjectComponent* _clone(QObject* newParent=nullptr) = 0;

public:
    /*!
     * \brief Constructs a component with a default name and a type
     * The default name is used to fill the "name" property of the component until
     * it is reassigned; the type identifies to the UI what group in the toolbox
     * should contain the component
     *
     * \param[in] defaultName Starting name of the component
     * \param[in] type Toolbox group of the component
     * \param[in] parent QObject parent
     */
    WorldObjectComponent(QString defaultName = "", QString type = "", QObject* parent=nullptr);

    /*!
     * \brief Creates a complete copy of the WorldObjectComponent
     * Calls the _clone() virtual copy of the inheriting class, and then
     * assigns all of the currently set property values into that new
     * object
     *
     * \param[in] newParent QObject parent of the new object
     * \return A WorldObjectComponent* copy of self
     */
    WorldObjectComponent* clone(QObject* newParent = nullptr);

    /*!
     * \brief Returns the list of Models drawn to represent this component
     * This is called only once, shortly after construction and the results
     * are cached. The set of models returned here is the set of models
     * registered with registerModel().
     *
     * \return Pointers to all registered Models of the component
     */
    QVector<Model*> getModels(){ return _models; }

    /*!
     * \brief Gets observers for all properties of the component
     * Properties should be mapped by unique string identifiers which briefly
     * describe the property's purpose. They are returned in shared pointers
     * so that if the UI does not let go of property views right away, it
     * is not left with dangling pointers.
     *
     * \return A map of property observers for the component
     */
    QMap<QString, QSharedPointer<PropertyView>> getProperties();

    /*!
     * \brief Getter for the current name of the component
     * \return The value contained in the "name" Property
     */
    QString getName(){ return _objName.get().toString(); }

    /*!
     * \brief Getter for the default name of the component
     * \return The value passed to the constructor as defaultName
     */
    QString getDefaultName() { return _defaultName; }

    /*!
     * \brief Getter for the component type
     * \return The value passed to the constructor for type
     */
    QString getType(){ return _type; }

    /*
    void setType(QString pType){ _type = pType; }
    void setName(QString pName){ _objName.set(pName); }
    */

    /*!
     * \brief Moves the component some delta x and delta y
     * Use this function to move the component and all of its
     * components and/or bodies as a group, preserving nesting
     * and relative location
     *
     * \param[in] x Distance to move in x direction
     * \param[in] y Distance to move in y direction
     */
    void translate(double x, double y);

    /*!
     * \brief Rotates the component some delta angle
     * Use this function to rotate the component and all of its
     * components and/or bodies as a group, preserving nesting
     * and relative location
     *
     * \param[in] degrees Number of degrees to rotate
     */
    void rotate(double degrees);

    /*!
     * \brief Sets the ROS 2 node that should be used for any messaging
     * The ROS node is not guaranteed to exist during the entire lifetime
     * of a component, so this function is used to pass references to the
     * node to the component when it is created. If the node is destroyed, a
     * nullptr may be passed here. Only components which want to publish or subscribe
     * ROS 2 messages need to override this method
     *
     * \param[in] node The ROS 2 Node to use (Or nullptr)
     */
    virtual void setROSNode(std::shared_ptr<rclcpp::Node> node){}

    /*!
     * \brief Getter to check if the component needs ROS 2 channels
     * If the component returns false, the application may optimize by removing
     * calls to connectChannels() and disconnectChannels() for this component
     *
     * \return True it the component will ever want to publish or subscribe ROS 2 messages
     */
    virtual bool usesChannels(){return false;}

    /*!
     * \brief Gives the component access to the physics engine so that it can add itself to the world
     * Since the number and types of physics bodies needed by components may vary greatly, this function
     * is used to let components add themselves to the physics engine. Components may create as many bodies
     * and joints as necessary for operation; components may also store the reference to the physics world
     * and use it to create/destroy/modify bodies in the future. The reference will be valid until a call
     * to clearBodies(). All b2Bodies created should be registered through registerBody(); this will move the
     * body from local-space to world space of the component.
     *
     * The anchor body should be used to joint all bodies together; unless bodies are jointed to one another,
     * they should be jointed to the anchor body so that they move in the physics world as a group (unless,
     * of course, the body shouldn't move at all, or should be free-floating)
     *
     * It is recommended to do steps in the following order:
     *  * Create any necessary bodies
     *  * Position all bodies within the component local space
     *  * Register all bodies to move them to world space
     *  * Joint bodies to each other and the anchor body
     *
     * Components should not destroy bodies that they did not create. Components
     * will usually be pieces of a larger object in the world. They should assign any
     * Box2D fixtures to the filter group -objectId (negative the object id) in order to prevent overlapping
     * parts of a world object from causing collisions.
     *
     * \param[in] world The Box2D world to create bodies in
     * \param[in] oId The object id this component is part of
     * \param[in] anchor The body that this component should joint itself to
     */
    virtual void generateBodies(b2World* world, object_id oId, b2Body* anchor){}

    /*!
     * \brief Signals the component to destroy all bodies it has created
     * The component should use the b2World* passed in the last call of generateBodies()
     * to destroy all bodies created since then. Bodies should first be unregistered
     * using unregisterBody() so that the underlying Component does not hold dangling
     * pointers. After this call completes, the component should treat the b2World*
     * as an invalid reference.
     */
    virtual void clearBodies(){}

    /*!
     * \brief Gets the name of the plugin that generated the component
     * This should return the Qt Plugin IID value that identifies
     * the plugin which creates components of the same type as this one.
     * This string is used to re-create components when they are loaded
     * from files.
     *
     * If the component did not come from a plugin, this should return
     * an empty string
     *
     * \return The IID of the component's plugin
     */
    virtual QString getPluginName() = 0;

public slots:
    /*!
     * Slot triggered when the component should
     * start publishing on and listening to ROS 2 channels; if
     * it uses them
     */
    virtual void connectChannels(){}

    /*!
     * Slot triggered when the component should
     * stop publishing on and listening to ROS 2 channels; if
     * it uses them
     */
    virtual void disconnectChannels(){}

    /*!
     * \brief Slot to update the parent WorldObjectComponent when physics ticks
     * When this is called, the parent WorldObjectComponent will apply any forces and
     * read any physics properties needed, as well as forward the call to
     * its inheriting class through _worldTicked()
     *
     * \param[in] t The amount of time simulated in the tick
     */
    void worldTicked(const double t);

    /*!
     * \brief Slot to update the WorldObjectComponent and sync models with the simulation.
     * During this method, the parent WorldObjectComponent will update any Models tied
     * to Bodies so that they have the same global position. Then the component
     * will forward the call to its inheriting type using _syncModels()
     */
    void syncModels();

    /*!
     * \brief Sets a global coordinate frame for this component
     * Component locations are calculated through a string of Transformation
     * matrices; this call is used to set the transformation up to the point
     * of this component. The component will work in this frame as if it is the
     * global frame.
     *
     * When the transform changes, there are two options
     *  * The component adjusts the local coordinates of all its models and bodies
     * to be relative to this new transformation; this happens during simulation
     *  * The component adjusts the global coordinates of all its models and bodies
     * to be unchanged relative to the parent transform; this happens when the user
     * manually moves or rotates components
     *
     * \param[in] t The transformation matrix
     * \param[in] cascade If true, the component will recompute its local transform and
     * forward it to any children components as their new global transform
     */
    void setParentTransform(QTransform t, bool cascade);

signals:
    /*!
     * \brief Indicates that the global transformation of this component changed
     * When the user moves a component or the main physics body of the component moves,
     * the component will update its global position and emit this signal. If the
     * user is moving the component, cascade will be true; as all children components
     * should recompute their transforms and move to stay in the same relative location.
     *
     * \param[in] t The new transform
     * \param[in] cascade Whether or not children components should move to stay in the same relative location
     */
    void transformChanged(QTransform t, bool cascade);
};

