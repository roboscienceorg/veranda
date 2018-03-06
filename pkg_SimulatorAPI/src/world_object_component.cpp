#include <QObject>
#include <QString>
#include <QMap>
#include <QTransform>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/property.h"
#include "sdsmt_simulator/world_object_component.h"

const double EPSILON = 0.0001;

QTransform rotation(double rad)
{
    double sina = sin(rad);
    double cosa = cos(rad);

    return QTransform(cosa, sina, -sina, cosa, 0, 0);
}

double radians(const QTransform& t)
{
    return atan2(t.m12(), t.m22());
}

QTransform bodyTransform(const b2Body* b)
{
    return rotation(b->GetAngle()) * QTransform(1, 0, 0, 1, b->GetPosition().x, b->GetPosition().y);
}

WorldObjectComponent::WorldObjectComponent(QString defaultName, QObject* parent) : QObject(parent)
{
    _objName.set(defaultName);

    connect(&_locX, &Property::valueSet, [this](){
        double diff = _locX.get().toDouble() - localTransform.dx();
        translate(diff, 0);
    });

    connect(&_locY, &Property::valueSet, [this](){
        double diff = _locY.get().toDouble() - localTransform.dy();
        translate(0, diff);
    });

    connect(&_locTheta, &Property::valueSet, [this](){
        double rad = radians(localTransform);
        double diff = _locTheta.get().toDouble() - (rad < 0 ? PI*2 + rad : rad)*RAD2DEG;
        rotate(diff);
    });

    connect(&_globX, &Property::valueSet, [this](){
        double diff = _globX.get().toDouble() - worldTransform.dx();
        translate(diff, 0);
    });

    connect(&_globX, &Property::valueSet, [this](){
        double diff = _globY.get().toDouble() - worldTransform.dy();
        translate(0, diff);
    });

    connect(&_globX, &Property::valueSet, [this](){
        double rad = radians(worldTransform);
        double diff = _globTheta.get().toDouble() - (rad < 0 ? PI*2 + rad : rad)*RAD2DEG;
        rotate(diff);
    });
}

//Register bodies so that they will be handled
//during transforms to other object local space
//Adding representations to a body means that they will be updated
//to have the same location as the body each tick
void WorldObjectComponent::registerBody(b2Body* bod, const QVector<Model*>& reprentations)
{
    _bodies[bod] = reprentations;

    //Move body to it's location within local space of this body
    //which, in turn, is in the local space of its parent
    QTransform newLoc = bodyTransform(bod) * localTransform * worldTransform;
    bod->SetTransform(b2Vec2(newLoc.dx(), newLoc.dy()), radians(newLoc));
}

void WorldObjectComponent::unregisterBody(b2Body* bod)
{
    _bodies.remove(bod);
}

//Register models so they will be handled during
//transforms
void WorldObjectComponent::registerModel(Model* mod)
{
    _models.insert(mod);
}

void WorldObjectComponent::unregisterModel(Model* mod)
{
    _models.remove(mod);
}

void WorldObjectComponent::adjustTransform(const QTransform& tOldI, const QTransform& tNew)
{
    QTransform tDiff = tOldI * tNew;

    for(b2Body* b : _bodies.keys())
    {
        //Multiply by inverse of old transform to get local space location,
        //then multiply by new transform to get new location
        QTransform newLoc = bodyTransform(b) * tDiff;
        b->SetTransform(b2Vec2(newLoc.dx(), newLoc.dy()), radians(newLoc));
    }
}

void WorldObjectComponent::updateProperties()
{
    _locX.set(localTransform.dx());
    _locY.set(localTransform.dy());
    _locTheta.set(radians(localTransform) * RAD2DEG);

    _globX.set(worldTransform.dx());
    _globY.set(worldTransform.dy());
    _globTheta.set(radians(worldTransform) * RAD2DEG);
}

void WorldObjectComponent::translate(double x, double y)
{
    if(abs(x) < EPSILON && abs(y) < EPSILON) return;

    localTransform.translate(x, y);
    QTransform tNew = localTransform * worldTransform;

    adjustTransform(invTransform, tNew);
    invTransform = tNew.inverted();

    emit transformChanged(tNew);

    updateProperties();
}

void WorldObjectComponent::rotate(double degrees)
{
    if(abs(degrees) < EPSILON) return;

    localTransform.rotate(degrees);
    QTransform tNew = localTransform * worldTransform;

    adjustTransform(invTransform, tNew);
    invTransform = tNew.inverted();

    emit transformChanged(tNew);

    updateProperties();
}

void WorldObjectComponent::setParentTransform(const QTransform& t)
{
    worldTransform = t;
    QTransform tNew = localTransform * worldTransform;

    adjustTransform(invTransform, tNew);
    invTransform = tNew.inverted();

    emit transformChanged(tNew);

    updateProperties();
}

void WorldObjectComponent::worldTicked(const double t)
{
    syncModels();
}

void WorldObjectComponent::syncModels()
{
    for(auto iter = _bodies.begin(); iter != _bodies.end(); iter++)
    {
        if(iter.value().size())
        {
            QTransform localLoc = bodyTransform(iter.key()) * invTransform;
            double x = localLoc.dx();
            double y = localLoc.dy();
            double t = radians(localLoc)*RAD2DEG;

            for(Model* m : iter.value())
                m->setTransform(x, y, t);
        }
    }

    _syncModels();
}

QMap<QString, QSharedPointer<PropertyView>> WorldObjectComponent::getProperties()
{
    QMap<QString, QSharedPointer<PropertyView>> out = _getProperties();

    for(auto iter = _properties.begin(); iter != _properties.end(); iter++)
        out[iter.key()] = QSharedPointer<PropertyView>(new PropertyView(iter.value()));

    return out;
}

void WorldObjectComponent::getTransform(double& x, double& y, double& degrees)
{
    x = localTransform.dx();
    y = localTransform.dy();
    degrees = radians(localTransform) * RAD2DEG;
}
