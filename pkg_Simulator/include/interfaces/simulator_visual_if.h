#ifndef SIMULATOR_VISUAL_H
#define SIMULATOR_VISUAL_H

#include <QWidget>

#include "sdsmt_simulator/model.h"
#include "world_object_if.h"

class Simulator_Visual_If : public QWidget
{
    Q_OBJECT

public:
    Simulator_Visual_If(QWidget* parent = nullptr) : QWidget(parent){}

    virtual void setWorldBounds(double xMin, double xMax, double yMin, double yMax) = 0;
signals:
    //Signals that the user clicked on a robot
    void userSelectedObject(object_id id);

public slots:
    virtual void objectAddedToScreen(QVector<Model*> objects, object_id oId) = 0;
    virtual void objectRemovedFromScreen(object_id id) = 0;
    virtual void objectDisabled(object_id id) = 0;
    virtual void objectEnabled(object_id id) = 0;
    virtual void objectSelected(object_id id) = 0;
};
#endif // SIMULATOR_VISUAL_H
