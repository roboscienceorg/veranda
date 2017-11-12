#ifndef BASIC_VIEWER_H
#define BASIC_VIEWER_H

#include "interfaces/simulator_visual_if.h"
#include "sdsmt_simulator/model.h"

#include <QMap>
#include <QTimer>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QLayout>

#include <Box2D/Box2D.h>

constexpr int64_t WORLD_SCALE = 20;

class BasicViewer : public Simulator_Visual_If
{
    Q_OBJECT

    QMap<object_id, QVector<Model*>> _models;
    QMap<Model*, QGraphicsItem*> _shapes;

    QTimer _refresh;

    QGraphicsView* _viewer;
    QGraphicsScene* _scene;
    QLayout* _children;

    QGraphicsItem* drawb2Shape(b2Shape* s, QGraphicsItem* itemParent = nullptr);
    QGraphicsItem* drawModel(Model* m, QGraphicsItem* parent=nullptr);
    void rescale();

    QPointF mouseClickPosition;

public:
    BasicViewer(QWidget* parent = nullptr);

    void setWorldBounds(double xMin, double xMax, double yMin, double yMax);
public slots:
    void objectAddedToScreen(QVector<Model *> objects, object_id id) override;
    void objectRemovedFromScreen(object_id id) override;
    void objectDisabled(object_id id) override;
    void objectEnabled(object_id id) override;
    void objectSelected(object_id id) override;

private slots:
    void modelMoved(Model* m, double dx, double dy, double dt);
    void modelChanged(Model* m);

    void mousePressEvent(QMouseEvent *event);
    void resizeEvent(QResizeEvent* event);
};

#endif // BASIC_VIEWER_H
