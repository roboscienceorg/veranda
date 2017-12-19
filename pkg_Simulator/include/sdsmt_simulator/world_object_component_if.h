#ifndef WORLD_OBJECT_COMPONENT_H
#define WORLD_OBJECT_COMPONENT_H

#include <QObject>
#include <QString>
#include <QMap>

#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/property.h"

class WorldObjectComponent_If : public QObject
{
    Q_OBJECT

public:
    WorldObjectComponent_If(QObject* parent=nullptr) : QObject(parent){}

    //Constructs copy of component
    virtual WorldObjectComponent_If* clone(QObject* newParent=nullptr) = 0;

    //Drawing Interactions
    virtual QVector<Model*> getModels() = 0;

    //UI Interactions
    virtual QMap<QString, PropertyView> getProperties() = 0;
    virtual QString getPropertyGroup() = 0;

    virtual bool usesChannels() = 0;

public slots:
    virtual void connectChannels() = 0;
    virtual void disconnectChannels() = 0;
};

#endif // WORLD_OBJECT_COMPONENT_H
