//! \file

#include <QObject>
#include <QString>
#include <QMap>
#include <QTransform>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "veranda_core/api/model.h"
#include "veranda_core/api/property.h"
#include "veranda_core/api/world_object_component.h"

/*!
 * \brief Creates a QTransform from a point and angle
 * \param[in] pos Translation of the transform
 * \param[in] rad Angle of the transform (radians)
 * \return The translation and rotation combined into a transform matrix
 */
QTransform transform(const b2Vec2& pos, const double& rad)
{
    double sina = sin(rad);
    double cosa = cos(rad);

    return QTransform(cosa, sina, -sina, cosa, pos.x, pos.y);
}

/*!
 * \brief Retrieves the angle of rotation from a Transformation matrix
 * \param[in] t The transform matrix
 * \return The rotation represented in the transform in radians
 */
double radians(const QTransform& t)
{
    return atan2(t.m12(), t.m22());
}

/*!
 * \brief Creates a transformation matrix representing the location of b2Body
 * \param[in] b The body to get a transformation from
 * \return The body's location as a transformation matrix
 */
QTransform bodyTransform(const b2Body* b)
{
    return transform(b->GetPosition(), b->GetAngle());
}

WorldObjectComponent::WorldObjectComponent(QString defaultName, QString type, QObject* parent) : QObject(parent)
{
    _objName.set(defaultName);
    _defaultName = defaultName;
    _type = type;

    //If any of the data values are adjusted manually,
    //we change our global location by the difference
    //and then update our local location
    connect(&_locX, &Property::valueRequested, [this](){
       //qDebug() << "Local translation";
        double diff = _locX.get().toDouble() - localTransform.dx();
        translate(diff, 0);
    });

    connect(&_locY, &Property::valueRequested, [this](){
       //qDebug() << "Local translation";
        double diff = _locY.get().toDouble() - localTransform.dy();
        translate(0, diff);
    });

    connect(&_locTheta, &Property::valueRequested, [this](){
       //qDebug() << "Local rotation";
        double rad = radians(localTransform);
        double diff = _locTheta.get().toDouble() - (rad < 0 ? PI*2 + rad : rad)*RAD2DEG;
        rotate(diff);
    });

    connect(&_globX, &Property::valueRequested, [this](){
        //qDebug() << "Global translation";
        double diff = _globX.get().toDouble() - globalPos.x;
        translate(diff, 0);
    });

    connect(&_globY, &Property::valueRequested, [this](){
        //qDebug() << "Global translation";
        double diff = _globY.get().toDouble() - globalPos.y;
        translate(0, diff);
    });

    connect(&_globTheta, &Property::valueRequested, [this](){
        //qDebug() << "Global rotation";
        double rad = globalRadians;
        double diff = _globTheta.get().toDouble() - rad*RAD2DEG;
        rotate(diff);
    });
}

WorldObjectComponent* WorldObjectComponent::clone(QObject* newParent)
{
    WorldObjectComponent* out = _clone(newParent);

    auto props = getProperties();
    auto outProps = out->getProperties();
    for(QString s : props.keys())
        if(outProps.contains(s) && props[s]->info().shouldSave)
        {
            outProps[s]->set(props[s]->get(), true);
        }
    return out;
}

/*!
 * The position of an added body is assumed to be its location relative to the
 * WorldObjectComponent origin.
 *
 * When a body is added, it will be moved in world-space
 * so that it is in the correct location relative to the component origin.
 * Any time the component moves, all bodies registered will be moved to maintain their
 * constraint.
 *
 * Models can also be added and tied to a body. Tying a model to a body will keep its
 * global location the same as the body's. It is not required that models tied to
 * a body be registered with registerModel()
 *
 * One body can be registered as the Main Body of the component. During the simulation,
 * the location of the main body will be used to update the position of the component.
 * If multiple main bodies are added, only the last one will be used as the main body
 *
 */
void WorldObjectComponent::registerBody(b2Body* bod, const QVector<Model*>& representations, bool isMainBody)
{
    _bodies[bod] = representations;

    b2Vec2 start = bod->GetPosition();

    //Move body to it's location within local space of this body
    //which, in turn, is in the local space of its parent
    QTransform newLoc = bodyTransform(bod) * globalTransform;
    bod->SetTransform(b2Vec2(newLoc.dx(), newLoc.dy()), radians(newLoc));

    //qDebug() << this << "Move body (" << start.x << start.y <<") -> (" << bod->GetPosition().x << bod->GetPosition().y << ")";
    //qDebug() << newLoc * invTransform;

    if(isMainBody)
    {
        _mainBody = bod;
        updateProperties();
    }

    //qDebug() << "Body added to" << this << bod;
    //qDebug() << _bodies.keys();
}

/*!
 * If the body removed is the main body, then there will no longer
 * be a main body and the position of the component will not update
 * when the simulation runs. This may lead to strange behavior; so only
 * remove the main body if you plan to put in a new one or the component
 * is being removed from simulation
 *
 * If the body had models tied to it, those constraints will be
 * broken as well, and the models will no longer auto-update to the
 * body's location
 */
void WorldObjectComponent::unregisterBody(b2Body* bod)
{
    _bodies.remove(bod);
    if(_mainBody == bod)
    {
        _mainBody = nullptr;
        updateProperties();
    }
    //qDebug() << "Body removed from" << this << bod;
    //qDebug() << _bodies.keys();
}

/*!
 * The position of an added model is assumed to be its location relative to the
 * WorldObjectComponent origin.
 *
 * When a body is model, it will be moved in world-space
 * so that it is in the correct location relative to the component origin.
 * Any time the component moves, all bodies registered will be moved to maintain their
 * constraint.
 *
 * Models are only moved directly if no bodies are registered with the component.
 * In this case, all models are moved together to keep their relative constraints.
 * As soon as any bodies are registered in the model, this behavior ceases, and
 * models will only move if they are tied to a registered body
 *
 * IMPORTANT!
 *
 * All top-level models should be constructed and regiestered during construction of
 * the component. Models registered after construction are not guaranteed
 * to be drawn; however, models may be added/removed as children of registered
 * models at any time
 */
void WorldObjectComponent::registerModel(Model* mod)
{
    _models.removeAll(mod);
    _models += mod;

    double x, y, t;
    mod->getTransform(x, y, t);
    QTransform newLoc = transform(b2Vec2(x, y), t*DEG2RAD) * globalTransform;

    mod->setTransform(newLoc.dx(), newLoc.dy(), radians(newLoc)*RAD2DEG);
}

void WorldObjectComponent::unregisterModel(Model* mod)
{
    _models.removeAll(mod);
}

void WorldObjectComponent::registerChild(WorldObjectComponent* child)
{
    if(!_children.contains(child))
    {
        _children.insert(child);

        //Connect child to our location; their current location is now where they stay relative to us
        connect(this, &WorldObjectComponent::transformChanged, child, &WorldObjectComponent::setParentTransform);
        child->setParentTransform(globalTransform, false);

        //Register all child models as our own
        for(Model* m : child->getModels())
            registerModel(m);
    }
}

void WorldObjectComponent::unregisterChild(WorldObjectComponent* child)
{
    if(_children.remove(child))
    {
        //Stop updating child location when we move
        disconnect(this, &WorldObjectComponent::transformChanged, child, &WorldObjectComponent::setParentTransform);

        //Unregister child models
        for(Model* m : child->getModels())
            unregisterModel(m);
    }
}

void WorldObjectComponent::shiftComponent(const QTransform& tOldI, const QTransform& tNew)
{
    QTransform tDiff = tOldI * tNew;
    //If no bodies exist, we shift all of the models
    //and retain their relative positions
    if(!_bodies.size())
    {
        for(Model* m :_models)
        {
            double x, y, t;
            m->getTransform(x, y, t);
            QTransform newLoc = transform(b2Vec2(x, y), t*DEG2RAD) * tDiff;

            m->setTransform(newLoc.dx(), newLoc.dy(), radians(newLoc)*RAD2DEG);
        }
    }
    //If bodies do exist, we shift them instead and
    //then later the models will be synchronized with them
    else
    {
        for(b2Body* b : _bodies.keys())
        {
            //Multiply by inverse of old transform to get local space location,
            //then multiply by new transform to get new location
            QTransform newLoc = bodyTransform(b) * tDiff;

            //qDebug() << this << "Move body (" << start.x << start.y <<") -> (" << b->GetPosition().x << b->GetPosition().y << ")";

            b->SetTransform(b2Vec2(newLoc.dx(), newLoc.dy()), radians(newLoc));
        }
    }
}

void WorldObjectComponent::updateProperties()
{
    _locX.set(localTransform.dx());
    _locY.set(localTransform.dy());
    _locTheta.set(radians(localTransform) * RAD2DEG);

    _globX.set(globalPos.x);
    _globY.set(globalPos.y);
    _globTheta.set(globalRadians * RAD2DEG);
}

void WorldObjectComponent::translate(double x, double y)
{
    if(std::abs(x) < EPSILON && std::abs(y) < EPSILON) return;

    globalPos.x += x;
    globalPos.y += y;
    //qDebug() << this << "adjust location by " << x << y << " : " << globalPos.x << globalPos.y;

    QTransform unGlobal = globalTransform.inverted();
    globalTransform = transform(globalPos, globalRadians);

    //Calculate local tranform from current parent
    if(hasParent)
        localTransform = globalTransform * parentInverse;

    //Move bodies by going from old global to new global
    shiftComponent(unGlobal, globalTransform);

    //Update children of global location;
    //and have them move to preserve their local transform
    transformChanged(transform(globalPos, globalRadians), true);

    syncModels();
    updateProperties();
}

void WorldObjectComponent::rotate(double degrees)
{
    if(std::abs(degrees) < EPSILON) return;

    globalRadians += degrees*DEG2RAD;
    if(globalRadians > 2*PI)
        globalRadians -= 2*PI*((int)globalRadians/(2*PI));
    else if(globalRadians < 0)
        globalRadians += 2*PI*((int)globalRadians/(2*PI) + 1);

    QTransform unGlobal = globalTransform.inverted();
    globalTransform = transform(globalPos, globalRadians);

    //Calculate local tranform from current parent
    if(hasParent)
        localTransform = globalTransform * parentInverse;

    //Move bodies by going from old global to new global
    shiftComponent(unGlobal, globalTransform);

    //Update children of global location;
    //and have them move to preserve their local transform
    transformChanged(globalTransform, true);

    syncModels();
    updateProperties();
}

void WorldObjectComponent::setParentTransform(QTransform t, bool cascade)
{
    hasParent = true;

    //Update parent's global transform
    parentTransform = t;
    parentInverse = t.inverted();
    //qDebug() << this << "Parent transform changed";

    //If cascading, then move self
    //so that our local transform doesn't change
    if(cascade)
    {
        //qDebug() << "Cascading; local transform is" << localTransform.dx() << localTransform.dy() << radians(localTransform) * RAD2DEG;

        //Find transform from global to new localspace
        //and from old localspace to global
        QTransform unGlobal = globalTransform.inverted();
        globalTransform = localTransform * parentTransform;

       //qDebug() << this << "Parent moved";

        //Move bodies by going from old local to global
        //then to new local
        shiftComponent(unGlobal, globalTransform);

        //Update models
        syncModels();

        //Update transform to children; have them
        //update
        emit transformChanged(globalTransform, true);
    }
    //If not cascading, adjust our local transform
    //so our global doesn't change
    else
    {
        //Recalculate our local tranform from the new parent location
        localTransform = globalTransform * parentInverse;
        //qDebug() << "Not Cascading; local transform is" << localTransform.dx() << localTransform.dy() << radians(localTransform) * RAD2DEG;
    }

    //Update local and global views
    updateProperties();
}

void WorldObjectComponent::worldTicked(const double t)
{
    for(WorldObjectComponent* c : _children)
        c->worldTicked(t);

    _worldTicked(t);
}

void WorldObjectComponent::syncModels()
{
    //For every model tied to a body
    //update it's transform to that body's
    //global location
    for(auto iter = _bodies.begin(); iter != _bodies.end(); iter++)
    {
        if(iter.value().size())
        {
            b2Vec2 pos = iter.key()->GetPosition();
            double t = iter.key()->GetAngle()*RAD2DEG;

            //qDebug() << this << "Sync models to body at " << pos.x << pos.y << t;

            for(Model* m : iter.value())
                m->setTransform(pos.x, pos.y, t);
        }
    }

    //If there's a main body, update
    //our global transform to match it
    //If we have a parent, recalculate our local transform
    //Publish our new global location to children, and have
    //them update their local location
    if(!_mainBody)
    {
        //qDebug() << "No main body defined for component " << this;
    }
    else
    {
        //Calculate global transform
        globalPos = _mainBody->GetPosition();
        globalRadians = _mainBody->GetAngle();

        if(globalRadians > 2*PI)
            globalRadians -= 2*PI*((int)globalRadians/(2*PI));
        else if(globalRadians < 0)
            globalRadians += 2*PI*((int)globalRadians/(2*PI) + 1);

        globalTransform = transform(globalPos, globalRadians);

        //Calculate local tranform from current parent
        if(hasParent)
            localTransform = globalTransform * parentInverse;

        //Update children of global location;
        //but don't trigger them to shift
        transformChanged(transform(globalPos, globalRadians), false);
    }

    for(WorldObjectComponent* c : _children)
        c->syncModels();

    _syncModels();
    updateProperties();
}

QMap<QString, QSharedPointer<PropertyView>> WorldObjectComponent::getProperties(bool includeChildren)
{
    QMap<QString, QSharedPointer<PropertyView>> out = _getProperties();

    for(auto iter = _properties.begin(); iter != _properties.end(); iter++)
        out[iter.key()] = QSharedPointer<PropertyView>(new PropertyView(iter.value()));

    if(includeChildren)
    {
        QMap<QString, int> groupcounts;

        //Quick tally of if any components use the same group name
        for(WorldObjectComponent* c : _children)
        {
            groupcounts[c->getName()]++;
        }

        QSet<QString> multiples;
        for(auto iter = groupcounts.begin(); iter != groupcounts.end(); iter++)
            if(iter.value() > 1) multiples.insert(iter.key());

        //Initialize all aggregates
        for(WorldObjectComponent* c : _children)
        {
            QMap<QString, QSharedPointer<PropertyView>> props = c->getProperties();

            //If more than 1 child uses group, append a number
            //to the group
            QString group = c->getName();

            if(multiples.contains(group))
                group += QString::number(groupcounts[group]--);

            //Add all properties to property map
            for(auto iter = props.begin(); iter != props.end(); iter++)
            {
                out[group + "/" + iter.key()] = iter.value();
            }
        }
    }

    return out;
}
