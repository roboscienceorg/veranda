#ifndef SIMULATOR_VISUAL_H
#define SIMULATOR_VISUAL_H

#include <QWidget>

#include "screen_model_if.h"

class Simulator_Visual_If : public QWidget
{
    Q_OBJECT

public:
    Simulator_Visual_If(QWidget* parent = nullptr) : QWidget(parent){}

signals:
    //Signals that the user clicked on a robot
    void userSelectedModel(model_id id);

public slots:
    virtual void modelAddedToScreen(ScreenModel_If* model, model_id id) = 0;
    virtual void modelRemovedFromScreen(model_id id) = 0;
    virtual void modelDisabled(model_id id) = 0;
    virtual void modelEnabled(model_id id) = 0;
    virtual void modelSelected(model_id id) = 0;
};
#endif // SIMULATOR_VISUAL_H
