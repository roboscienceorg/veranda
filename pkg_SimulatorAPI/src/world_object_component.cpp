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

    connect(&_locX, &Property::valueRequested, [this](){
       //qDebug() << "Local translation";
        double diff = _locX.get().toDouble() - localTranslate.dx();
        translate(diff, 0);
    });

    connect(&_locY, &Property::valueRequested, [this](){
       //qDebug() << "Local translation";
        double diff = _locY.get().toDouble() - localTranslate.dy();
        translate(0, diff);
    });

    connect(&_locTheta, &Property::valueRequested, [this](){
       //qDebug() << "Local rotation";
        double rad = radians(localRotate);
        double diff = _locTheta.get().toDouble() - (rad < 0 ? PI*2 + rad : rad)*RAD2DEG;
        rotate(diff);
    });

    connect(&_globX, &Property::valueRequested, [this](){
        //qDebug() << "Global translation";
        double diff = _globX.get().toDouble() - worldTransform.dx();
        translate(diff, 0);
    });

    connect(&_globY, &Property::valueRequested, [this](){
        //qDebug() << "Global translation";
        double diff = _globY.get().toDouble() - worldTransform.dy();
        translate(0, diff);
    });

    connect(&_globTheta, &Property::valueRequested, [this](){
        //qDebug() << "Global rotation";
        double rad = radians(worldTransform);
        double diff = _globTheta.get().toDouble() - (rad < 0 ? PI*2 + rad : rad)*RAD2DEG;
        rotate(diff);
    });

    _masterModel = new Model();
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
void WorldObjectComponent::registerBody(b2Body* bod, const QVector<Model*>& reprentations)
{
    _bodies[bod] = reprentations;

    //Move body to it's location within local space of this body
    //which, in turn, is in the local space of its parent
    QTransform newLoc = bodyTransform(bod) * localRotate * localTranslate * worldTransform;
    bod->SetTransform(b2Vec2(newLoc.dx(), newLoc.dy()), radians(newLoc));
    //qDebug() << "Body added to" << this << bod;
    //qDebug() << _bodies.keys();
}

void WorldObjectComponent::unregisterBody(b2Body* bod)
{
    _bodies.remove(bod);
    //qDebug() << "Body removed from" << this << bod;
    //qDebug() << _bodies.keys();
}

//Register models so they will be handled during
//transforms
void WorldObjectComponent::registerModel(Model* mod)
{
    _masterModel->addChildren({mod});
}

void WorldObjectComponent::unregisterModel(Model* mod)
{
    _masterModel->removeChildren({mod});
}

void WorldObjectComponent::adjustTransform(const QTransform& tOldI, const QTransform& tNew)
{
    QTransform tDiff = tOldI * tNew;

    for(b2Body* b : _bodies.keys())
    {
        b2Vec2 start = b->GetPosition();
        //Multiply by inverse of old transform to get local space location,
        //then multiply by new transform to get new location
        QTransform newLoc = bodyTransform(b) * tDiff;
        //qDebug() << "Move body (" << start.x << start.y <<") -> (" << b->GetPosition().x << b->GetPosition().y << ")";
        b->SetTransform(b2Vec2(newLoc.dx(), newLoc.dy()), radians(newLoc));
    }

    syncModels();
}

void WorldObjectComponent::updateProperties()
{
    _locX.set(localTranslate.dx());
    _locY.set(localTranslate.dy());
    _locTheta.set(radians(localRotate) * RAD2DEG);

    QTransform globalTransform = localRotate * localTranslate * worldTransform;
    _globX.set(globalTransform.dx());
    _globY.set(globalTransform.dy());
    _globTheta.set(radians(globalTransform) * RAD2DEG);
}

void WorldObjectComponent::translate(double x, double y)
{
    if(std::abs(x) < EPSILON && std::abs(y) < EPSILON) return;

    double tmpx = localTranslate.dx(), tmpy = localTranslate.dy();
    localTranslate.translate(x, y);
   //qDebug() << this << "Translate" << x << y << " from " << tmpx << tmpy << " to " << localTranslate.dx() << localTranslate.dy();
    QTransform tNew = localRotate * localTranslate * worldTransform;

    updateProperties();

    //If any bodies exist, physically move them and then update the models to them
    //Otherwise, move the models
    if(_bodies.size())
        adjustTransform(invTransform, tNew);
    else
        _masterModel->setTransform(_locX.get().toDouble(), _locY.get().toDouble(), _locTheta.get().toDouble() * RAD2DEG);

    bool inverted;
    invTransform = tNew.inverted(&inverted);
    if(!inverted)qDebug() << "Failed to invert";

    emit transformChanged(tNew);
}

void WorldObjectComponent::rotate(double degrees)
{
    if(std::abs(degrees) < EPSILON) return;

    localRotate.rotate(degrees);
    QTransform tNew = localRotate * localTranslate * worldTransform;

    updateProperties();

    //If any bodies exist, physically move them and then update the models to them
    //Otherwise, move the models
    if(_bodies.size())
        adjustTransform(invTransform, tNew);
    else
        _masterModel->setTransform(_locX.get().toDouble(), _locY.get().toDouble(), _locTheta.get().toDouble() * RAD2DEG);

    bool inverted;
    invTransform = tNew.inverted(&inverted);
    if(!inverted)qDebug() << "Failed to invert";

    emit transformChanged(tNew);

}

void WorldObjectComponent::setParentTransform(const QTransform& t)
{
    worldTransform = t;
    QTransform tNew = localRotate * localTranslate * worldTransform;
   //qDebug() << this << "Parent moved";
    adjustTransform(invTransform, tNew);

    bool inverted;
    invTransform = tNew.inverted(&inverted);
    if(!inverted)qDebug() << "Failed to invert";

    emit transformChanged(tNew);

    updateProperties();
}

void WorldObjectComponent::worldTicked(const double t)
{
    _worldTicked(t);
}

void WorldObjectComponent::syncModels()
{
    _masterModel->setTransform(0, 0, 0);
    QTransform diff = invTransform * localRotate * localTranslate;
    for(auto iter = _bodies.begin(); iter != _bodies.end(); iter++)
    {
        if(iter.value().size())
        {
            QTransform localLoc = bodyTransform(iter.key()) * diff;
            double x = localLoc.dx();
            double y = localLoc.dy();
            double t = radians(localLoc)*RAD2DEG;

           //qDebug() << this << "Sync model to body at" << iter.key()->GetPosition().x << iter.key()->GetPosition().y << ":" << x << y;

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
    x = localTranslate.dx();
    y = localTranslate.dy();
    degrees = radians(localRotate) * RAD2DEG;
}
