#ifndef BASIC_VIEWER_H
#define BASIC_VIEWER_H

#include "interfaces/simulator_visual_if.h"

class BasicViewer : public Simulator_Visual_If
{
    Q_OBJECT

public:
    BasicViewer(QWidget* parent = nullptr);

public slots:
    void modelAddedToScreen(ScreenModel_If* model, model_id id) override;
    void modelRemovedFromScreen(model_id id) override;
    void modelDisabled(model_id id) override;
    void modelEnabled(model_id id) override;
    void modelSelected(model_id id) override;
};

#endif // BASIC_VIEWER_H
