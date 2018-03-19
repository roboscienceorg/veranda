#include <QObject>
#include <QString>
#include <QMap>
#include <QTransform>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/property.h"
#include "sdsmt_simulator/world_object_component.h"

QTransform transform(const b2Vec2& pos, const double& rad)
{
    double sina = sin(rad);
    double cosa = cos(rad);

    return QTransform(cosa, sina, -sina, cosa, pos.x, pos.y);
}

double radians(const QTransform& t)
{
    return atan2(t.m12(), t.m22());
}

QTransform bodyTransform(const b2Body* b)
{
    return transform(b->GetPosition(), b->GetAngle());
}

WorldObjectComponent::WorldObjectComponent(QString defaultName, QObject* parent) : QObject(parent)
{
    _objName.set(defaultName);

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
        outProps[s]->set(props[s]->get(), true);

    return out;
}

//Register bodies so that they will be handled
//during transforms to other object local space
//Adding representations to a body means that they will be updated
//to have the same location as the body each tick
void WorldObjectComponent::registerBody(b2Body* bod, const QVector<Model*>& reprentations, bool isMainBody)
{
    _bodies[bod] = reprentations;

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

//Register models so they will be handled during
//transforms
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
    qDebug() << this << "adjust location by " << x << y << " : " << globalPos.x << globalPos.y;

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
    _worldTicked(t);
}

void WorldObjectComponent::syncModels()
{
    //Nothing to do if no bodies are registered
    if(!_bodies.size())
    {
        _syncModels();
        return;
    }

    qDebug() << "syncing models ";
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

    _syncModels();
    updateProperties();
}

QMap<QString, QSharedPointer<PropertyView>> WorldObjectComponent::getProperties()
{
    QMap<QString, QSharedPointer<PropertyView>> out = _getProperties();

    for(auto iter = _properties.begin(); iter != _properties.end(); iter++)
        out[iter.key()] = QSharedPointer<PropertyView>(new PropertyView(iter.value()));

    return out;
}
