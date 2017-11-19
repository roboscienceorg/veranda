#ifndef ROBOTCOMPONENT_IF_H
#define ROBOTCOMPONENT_IF_H

#include "property.h"

#include <QObject>

#include <Box2D/Box2D.h>

#include "model.h"
#include "interfaces/old_world_object_if.h"

class RobotComponent_If : public depracatedWorldObject_If
{
    Q_OBJECT

public:
    RobotComponent_If(QObject* parent=nullptr) : depracatedWorldObject_If(parent){}

signals:
    void massChanged();
};


#endif // ROBOTCOMPONENT_IF_H
