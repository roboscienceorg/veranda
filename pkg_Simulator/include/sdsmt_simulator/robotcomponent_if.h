#ifndef ROBOTCOMPONENT_IF_H
#define ROBOTCOMPONENT_IF_H

#include "properties_if.h"

#include <QObject>

#include <Box2D/Box2D.h>

#include "model.h"
#include "interfaces/world_object_if.h"

class RobotComponent_If : public WorldObject_If
{
    Q_OBJECT

public:
    RobotComponent_If(QObject* parent=nullptr) : WorldObject_If(parent){}

signals:
    void massChanged();
};


#endif // ROBOTCOMPONENT_IF_H
