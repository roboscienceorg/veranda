#ifndef WORLD_OBJECT_COMPONENT_H
#define WORLD_OBJECT_COMPONENT_H

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

class SDSMT_SIMULATOR_API WorldObjectComponent : public QObject
{
    Q_OBJECT

    Property _objName;
    Property _locX = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "X coord of the object relative to its parent"),
                                QVariant(0.0), &Property::double_validator);

    Property _locY = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Y coord of the object relative to its parent"),
                                QVariant(0.0), &Property::double_validator);

    Property _locTheta = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Angle of the object (Degrees) relative to its parent"),
                                QVariant(0.0), &Property::angle_validator);

    Property _globX = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Global X coord of the object"),
                                QVariant(0.0), &Property::double_validator);

    Property _globY = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Global Y coord of the object"),
                                QVariant(0.0), &Property::double_validator);

    Property _globTheta = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE, "Global Angle of the object (Degrees)"),
                                QVariant(0.0), &Property::angle_validator);

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

    b2Body* _mainBody = nullptr;
    QVector<Model*> _models;
    QMap<b2Body*, QVector<Model*>> _bodies;

    QTransform parentTransform, parentInverse;
    QTransform localTransform;
    QTransform globalTransform;
    double globalRadians = 0;
    b2Vec2 globalPos = b2Vec2(0, 0);
    bool hasParent = false;

    void shiftComponent(const QTransform& tOldI, const QTransform& tNew);
    void updateProperties();

protected:
    //Register bodies so that they will be handled
    //during transforms to other object local space
    //Adding representations to a body means that they will be updated
    //to have the same location as the body each tick
    void registerBody(b2Body* bod, const QVector<Model*>& reprentations = {}, bool isMainBody = false);
    void unregisterBody(b2Body* bod);

    //Register models so they will be handled during
    //transforms
    void registerModel(Model* mod);
    void unregisterModel(Model* mod);

    virtual QMap<QString, QSharedPointer<PropertyView>> _getProperties(){return {};}
    virtual void _worldTicked(const double t){}
    virtual void _syncModels(){}

public:
    WorldObjectComponent(QString defaultName = "", QObject* parent=nullptr);

    //Constructs copy of component
    virtual WorldObjectComponent* _clone(QObject* newParent=nullptr) = 0;
    WorldObjectComponent* clone(QObject* newParent = nullptr);

    //Drawing Interactions
    QVector<Model*> getModels(){ return _models; }

    //UI Interactions
    QMap<QString, QSharedPointer<PropertyView>> getProperties();
    QString getName(){ return _objName.get().toString(); }

    void translateView(double x, double y, double degrees);
    void translate(double x, double y);
    void rotate(double degrees);

    //Interactions with ROS
    virtual void setROSNode(std::shared_ptr<rclcpp::Node> node){}
    virtual bool usesChannels(){return false;}

    virtual void generateBodies(b2World* world, object_id oId, b2Body* anchor){}
    virtual void clearBodies(){}

public slots:
    virtual void connectChannels(){}
    virtual void disconnectChannels(){}

    void worldTicked(const double t);
    void syncModels();
    void setParentTransform(QTransform t, bool cascade);

protected slots:

signals:
    void transformChanged(QTransform t, bool cascade);
};

#endif // WORLD_OBJECT_COMPONENT_H
