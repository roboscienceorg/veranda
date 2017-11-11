#ifndef SENSOR_H
#define SENSOR_H

#include <Box2D/Box2D.h>

#include <QVector>
#include <QObject>

#include "robotcomponent_if.h"

class Sensor_If : public RobotComponent_If
{
    Q_OBJECT

public:
    Sensor_If(QObject* parent = nullptr) : RobotComponent_If(parent){}

signals:
    void visualChanged(QVector<b2Shape*> visualObjects);
};
#endif // SENSOR_H
