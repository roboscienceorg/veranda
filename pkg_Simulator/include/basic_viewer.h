#ifndef BASIC_VIEWER_H
#define BASIC_VIEWER_H

#include "interfaces/simulator_visual_if.h"

#include <QMap>
#include <QTimer>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QLayout>

#include <Box2D/Box2D.h>

class BasicViewer : public Simulator_Visual_If
{
    Q_OBJECT

    QMap<model_id, ScreenModel_If*> _models;
    QMap<ScreenModel_If*, QGraphicsEllipseItem*> _shapes;

    QTimer _refresh;

    QGraphicsView* _viewer;
    QGraphicsScene* _scene;
    QLayout* _children;

public:
    BasicViewer(QWidget* parent = nullptr);
    QPointF mouseClickPosition;

public slots:
    void modelAddedToScreen(ScreenModel_If* model, model_id id) override;
    void modelRemovedFromScreen(model_id id) override;
    void modelDisabled(model_id id) override;
    void modelEnabled(model_id id) override;
    void modelSelected(model_id id) override;

private slots:
    void modelMoved(ScreenModel_If* m);
    void mousePressEvent(QMouseEvent *event);
};

#endif // BASIC_VIEWER_H
