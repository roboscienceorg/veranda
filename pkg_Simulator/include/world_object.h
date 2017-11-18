#ifndef WORLD_OBJECT_IF_H
#define WORLD_OBJECT_IF_H

#include <QObject>
#include <QString>
#include <QMap>
#include <QVector>

#include "sdsmt_simulator/world_object_component_if.h"
#include "sdsmt_simulator/model.h"
#include "sdsmt_simulator/properties_if.h"

class WorldObject : public QObject
{
    Q_OBJECT

    QVector<WorldObjectComponent_If*> _components;

    bool _useChannels = false;

    Property _objName;

    QVector<Model*> _models;
    QMap<QString, PropertyView> _properties
    {
        {"name", &_objName}
    };

public:
    WorldObject(QVector<WorldObjectComponent_If*> components, QObject* parent = nullptr);

    //Constructs copy of object
    WorldObject* clone(QObject* newParent=nullptr);

    //Drawing Interactions
    QVector<Model*> getModels()
    { return _models; }

    //UI Interactions
    QMap<QString, PropertyView> getProperties()
    { return _properties; }

    bool usesChannels()
    { return _useChannels; }

    //Physics Interactions

public slots:
    void connectChannels();
    void disconnectChannels();
};

#endif // WORLD_OBJECT_IF_H
