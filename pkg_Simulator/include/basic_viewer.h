#ifndef BASIC_VIEWER_H
#define BASIC_VIEWER_H

#include "interfaces/simulator_visual_if.h"

#include <QMap>
#include <QTimer>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QLayout>
#include <QColor>

#include <Box2D/Box2D.h>
#include <sdsmt_simulator/model.h>

class BasicViewer : public Simulator_Visual_If
{
    Q_OBJECT

    constexpr static int64_t WORLD_SCALE = 20;

    //Actual models provided for object
    QMap<object_id, QVector<Model*>> _models;
    QMap<Model*, object_id> _modelToObject;

    //Top-level parent shape for objects
    QMap<Model*, QGraphicsItem*> _shapes;
    QMap<QGraphicsItem*, Model*> _shapeToModel;

    //Keep track of what is drawn
    QMap<object_id, DrawLevel> _drawLevels;

    //What is currently selected
    object_id _currSelection = 0;

    //Viewer, scene, and widget layout
    QGraphicsView* _viewer;
    QGraphicsScene* _scene;
    QLayout* _children;

    //Constructs a QAbstractGraphicsShapeItem from a box2d shape
    QGraphicsItem* _drawb2Shape(b2Shape* s, QGraphicsItem* itemParent = nullptr);

    //Constructs a QGraphicsItem with multiple shapes from a model
    QGraphicsItem* _drawModel(Model* m, QGraphicsItem* parent=nullptr);

    //Rescales the view
    void _rescale();

    //Returns a color given drawlevel and state of selected
    QColor _color(DrawLevel level, bool selected);

    //Sets a graphics item and all it's children to a specific color pen
    void _setOutlineColor(QGraphicsItem* item, const QColor& color);

public:
    BasicViewer(QWidget* parent = nullptr);

    void setWorldBounds(double xMin, double xMax, double yMin, double yMax);
public slots:
    void objectAddedToScreen(QVector<Model *> objects, object_id id) override;
    void objectRemovedFromScreen(object_id id) override;
    void objectDrawLevelSet(object_id id, DrawLevel level) override;
    void objectSelected(object_id id) override;

private slots:
    void modelMoved(Model* m, double dx, double dy, double dt);
    void modelChanged(Model* m);

    void mousePressEvent(QMouseEvent *event);
    void resizeEvent(QResizeEvent* event);
};

#endif // BASIC_VIEWER_H
