#include "basic_viewer.h"

#include <QPainter>
#include <QDebug>
#include <QVBoxLayout>
#include <QMouseEvent>
#include <QMessageBox>
#include <QQueue>

#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsItemGroup>

#include <QThread>

#include <cmath>

//ScreenModel_if - found in a header file

//Constructor
//Sets up widget with subwidget to view a graphics scene
//makes the
BasicViewer::BasicViewer(QWidget *parent) : Simulator_Visual_If(parent)
{
    _children = new QVBoxLayout(this);

    _scene = new QGraphicsScene(this);
    _viewer = new CustomGraphicsView(_scene, this);
    _viewer->setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding));
    _viewer->setMouseTracking(true);

    connect(_viewer, &CustomGraphicsView::mouseMoved, this, &BasicViewer::viewMouseMove);
    connect(_viewer, &CustomGraphicsView::mousePress, this, &BasicViewer::viewMousePress);
    connect(_viewer, &CustomGraphicsView::mouseRelease, this, &BasicViewer::viewMouseRelease);

    _children->addWidget(_viewer);
    setLayout(_children);

    _transformer = _makeTransformer();

    setWorldBounds(-200, 200, -200, 200);
}

QGraphicsItem* BasicViewer::_drawb2Shape(b2Shape* s, QGraphicsItem* itemParent)
{
    QGraphicsItem* newShape = nullptr;
    switch(s->m_type)
    {
        case b2Shape::Type::e_circle:
        {
            b2CircleShape* circle = static_cast<b2CircleShape*>(s);
            double r = circle->m_radius;
            newShape = new QGraphicsEllipseItem((circle->m_p.x-r) * WORLD_SCALE, -(circle->m_p.y+r) * WORLD_SCALE,
                                                circle->m_radius*2 * WORLD_SCALE, circle->m_radius*2 * WORLD_SCALE, itemParent);
        }
        break;
        case b2Shape::Type::e_edge:
        {
            b2EdgeShape* edge = static_cast<b2EdgeShape*>(s);
            newShape = new QGraphicsLineItem(edge->m_vertex1.x * WORLD_SCALE, -edge->m_vertex1.y * WORLD_SCALE,
                                             edge->m_vertex2.x * WORLD_SCALE, -edge->m_vertex2.y * WORLD_SCALE, itemParent);
        }
        break;
        case b2Shape::Type::e_polygon:
        {
            b2PolygonShape* poly = static_cast<b2PolygonShape*>(s);

            QPolygonF qpoly;
            for(b2Vec2* vert = poly->m_vertices; vert != poly->m_vertices + poly->m_count; vert++)
            {
                qpoly.append(QPointF(vert->x*WORLD_SCALE, -vert->y*WORLD_SCALE));
            }

            newShape = new QGraphicsPolygonItem(qpoly, itemParent);
        }
    }
    return newShape;
}

QGraphicsItem* BasicViewer::_drawModel(Model* m)
{
    QGraphicsItemGroup* baseItem = new QGraphicsItemGroup();

    //Draw all shapes as children of the base item
    for(b2Shape* s : m->shapes())
        baseItem->addToGroup(_drawb2Shape(s));

    //Set location of model
    double x, y, t;
    m->getTransform(x, y, t);

    baseItem->moveBy(x * WORLD_SCALE, -y*WORLD_SCALE);
    baseItem->setRotation(-t);

    return baseItem;
}

//Something new was added to the world view
//It needs to get added to the graphics scene and indexed
//When the model updates, (transformChanged) the graphics shape
//needs to be moved within the scene
void BasicViewer::objectAddedToScreen(QVector<Model*> objects, object_id id)
{
    _models[id] = objects;

    QColor newColor = _color(Solid, false);
    _drawLevels[id] = Solid;

    QGraphicsItemGroup* group = new QGraphicsItemGroup();
    for(Model* m : objects)
    {
        QGraphicsItem* graphic = addModel(m, id);

        group->addToGroup(graphic);
    }
    _setOutlineColor(group, newColor);
    _scene->addItem(group);

    _shapeToObject[group] = id;
    _topShapes[id] = group;
}

QGraphicsItem* BasicViewer::addModel(Model *m, object_id id)
{
    QGraphicsItem* graphic = _drawModel(m);

    _shapes[m] = graphic;
    _shapeToObject[graphic] = id;
    _modelToObject[m] = id;
    _modelChildren[m] = m->children();

    //If the model or one of its submodels changes, redraw the whole thing
    connect(m, &Model::modelChanged, this, &BasicViewer::modelChanged);

    //If the base model moves, update the transform
    connect(m, &Model::transformChanged, this, &BasicViewer::modelMoved);

    for(Model* child : m->children())
    {
        QGraphicsItem* cGraph = addModel(child, id);

        cGraph->setParentItem(graphic);
    }

    return graphic;
}


//Updates a graphics scene object to have a new
//location
void BasicViewer::modelMoved(Model *m, double dx, double dy, double dt)
{
    double x, y, t;
    m->getTransform(x, y, t);

    x *= WORLD_SCALE;
    y *= WORLD_SCALE;

    dx *= WORLD_SCALE;
    dy *= WORLD_SCALE;

    _shapes[m]->moveBy(dx, -dy);
    _shapes[m]->setRotation(-t);

    object_id oid = _modelToObject[m];
    if(_currSelection == oid)
        _transformer->moveBy(dx, -dy);
}

void BasicViewer::modelChanged(Model *m)
{
    object_id oid = _modelToObject[m];

    QGraphicsItem* parent = _shapes[m]->parentItem();

    if(parent == nullptr)
        _scene->removeItem(_shapes[m]);

    //Remove old version
    removeModel(m);

    //Draw and add new version
    QGraphicsItem* replace = addModel(m, oid);

    replace->setParentItem(parent);

    QColor newColor = _color(_drawLevels[oid], _currSelection == oid);
    _setOutlineColor(replace, newColor);

    if(parent == nullptr)
        _scene->addItem(replace);
}

//World View Clicked
void BasicViewer::viewMousePress(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        QPointF hit = event->localPos();

        bool startDrag = _transformer->contains(
                    _transformer->sceneTransform().inverted().map(
                        _viewer->mapToScene(
                            _viewer->mapFromGlobal(
                                event->globalPos()))));

        if(!_draggingTranslate && startDrag)
        {
            //qDebug() << "Start dragging";
            _draggingTranslate = true;
            _dragStart = _viewer->mapToScene(
                        _viewer->mapFromGlobal(
                            event->globalPos()));
        }
        else
        {
            QGraphicsItem* shape = _viewer->itemAt((int)(hit.x()+0.5), (int)(hit.y() + 0.5));
            while(shape && shape->parentItem())
                shape = shape->parentItem();
            object_id oid = _shapeToObject[shape];

            userSelectedObject(oid);
        }
    }
}

void BasicViewer::viewMouseMove(QMouseEvent* event)
{
    if(_draggingTranslate)
    {
        QPointF newLocal = _viewer->mapToScene(
                    _viewer->mapFromGlobal(
                        event->globalPos()));
        QPointF delta = (newLocal - _dragStart)/(WORLD_SCALE);
        userDragMoveObject(_currSelection, delta.x(), -delta.y());
        _dragStart = newLocal;
    }
}

void BasicViewer::viewMouseRelease(QMouseEvent *event)
{
    _draggingTranslate = false;
    _draggingRotate = false;
}

void BasicViewer::resizeEvent(QResizeEvent *event)
{
    _rescale();
}

void BasicViewer::_rescale()
{
    //It appears that QGraphicsView::fitInView is broken in
    //Qt 5.5. This should be an acceptable alternative
    //If we allow the user to pan and zoom, then this will
    //likely need to change to keep track of zoom and pan parameters
    //so they can correctly be part of this transform
    double w_acutal = geometry().width()*0.9;
    double h_actual = geometry().height()*0.9;

    double w_need = _scene->width();
    double h_need = _scene->height();

    double scale = std::min(w_acutal/w_need, h_actual/h_need);

    QTransform matrix;
    matrix.scale(scale,
                 scale);
    _viewer->setTransform(matrix);
}

void BasicViewer::setWorldBounds(double xMin, double xMax, double yMin, double yMax)
{
    QRect viewRect(xMin*WORLD_SCALE, -yMax*WORLD_SCALE, (xMax-xMin)*WORLD_SCALE, (yMax-yMin)*WORLD_SCALE);
    _scene->setSceneRect(viewRect);
    _viewer->setSceneRect(viewRect);

    _rescale();
}

//for after the MVP
//The object identified by object_id is no longer on the world
void BasicViewer::objectRemovedFromScreen(object_id id)
{
    if(_models.contains(id))
    {
        for(Model* m : _models[id])
        {
            _scene->removeItem(_shapes[m]);
            removeModel(m);
        }
        _models.remove(id);
        _drawLevels.remove(id);
        _shapeToObject.remove(_topShapes[id]);

        delete _topShapes[id];
        _topShapes.remove(id);
    }
    if(_currSelection == id) _currSelection = 0;
}

void BasicViewer::removeModel(Model *m)
{
    for(Model* child : _modelChildren[m])
        removeModel(child);

    //Stop drawing shapes
    _modelToObject.remove(m);
    _modelChildren.remove(m);
    _shapeToObject.remove(_shapes[m]);

    delete _shapes[m];

    //Stop tracking shape
    _shapes.remove(m);

    //Stop tracking model
    disconnect(m, 0, this, 0);

}

//The object identified by object_id is in the world, but should not be drawn
//for users not wanting to see their sensors drawn
void BasicViewer::objectDrawLevelSet(object_id id, DrawLevel level)
{
    _drawLevels[id] = level;
    QColor newColor = _color(level, _currSelection == id);
    for(Model* m : _models[id])
        _setOutlineColor(_shapes[m], newColor);
}

//The object identified by object_id has been selcted
//maybe we draw a selection box around it?
void BasicViewer::objectSelected(object_id id)
{
    QColor newColor;
    if(id != _currSelection)
    {
        nothingSelected();

        _currSelection = id;
        newColor = _color(_drawLevels[_currSelection], true);
        _setOutlineColor(_topShapes[id], newColor);

        QRectF bound = _topShapes[id]->boundingRect();
        double leftEdge = bound.x() + bound.width(), bottomEdge = bound.y() + bound.height();

        _transformer->setPos(leftEdge + _transformer->boundingRect().width()*0.5, bottomEdge + _transformer->boundingRect().height()*0.5);
        _scene->addItem(_transformer);
    }
}

//No objects are selected, draw without highlights
void BasicViewer::nothingSelected()
{
    QColor newColor;
    if(_currSelection != 0)
    {
        newColor = _color(_drawLevels[_currSelection], false);
        for(Model* m : _models[_currSelection])
            _setOutlineColor(_shapes[m], newColor);

        _currSelection = 0;
        _scene->removeItem(_transformer);
    }
}

//Returns a color given drawlevel and state of selected
QColor BasicViewer::_color(DrawLevel level, bool selected)
{
    QColor out(0, 0, 0);

    if(selected)
        out.setRgb(50, 163, 103);

    switch(level)
    {
        case Solid:
            out.setAlpha(255);
        break;
        case Transparent:
            out.setAlpha(125);
        break;
        case Off:
            out.setAlpha(0);
        break;
    }

    return out;
}

//Sets a graphics item and all it's children to a specific color pen
void BasicViewer::_setOutlineColor(QGraphicsItem* item, const QColor& color)
{
    QAbstractGraphicsShapeItem* asShape = dynamic_cast<QAbstractGraphicsShapeItem*>(item);

    if(asShape)
    {
        asShape->setPen(QPen(color));
    }
    else
    {
        //QGraphicsLine is not a QAbstractGraphicsShapeItem so it needs to be a special check
        QGraphicsLineItem* asLine = dynamic_cast<QGraphicsLineItem*>(item);
        if(asLine)
            asLine->setPen(QPen(color));
    }

    for(QGraphicsItem* i : item->childItems())
        _setOutlineColor(i, color);
}

QGraphicsItem* BasicViewer::_makeTransformer()
{
    QBrush b(QColor(50, 163, 103));
    QPen p(QColor(50, 163, 103));

    QGraphicsItemGroup* group = new QGraphicsItemGroup;

    QGraphicsRectItem* rect = new QGraphicsRectItem(0.0, -1.0*WORLD_SCALE, 0.5*WORLD_SCALE, 2.5*WORLD_SCALE);
    rect->setBrush(b);
    rect->setPen(p);
    group->addToGroup(rect);

    rect = new QGraphicsRectItem(-1.0*WORLD_SCALE, 0.0, 2.5*WORLD_SCALE, 0.5*WORLD_SCALE);
    rect->setBrush(b);
    rect->setPen(p);

    group->addToGroup(rect);
    group->addToGroup(_makeArrow(.25, -1.3, 0, p, b));
    group->addToGroup(_makeArrow(.25, 1.8, 180, p, b));
    group->addToGroup(_makeArrow(1.8, .25, 90, p, b));
    group->addToGroup(_makeArrow(-1.3, .25, 270, p, b));

    return group;
}

QGraphicsItem* BasicViewer::_makeRotater()
{
    return nullptr;
}

QGraphicsItem* BasicViewer::_makeArrow(double pointx, double pointy, double angle, QPen p, QBrush b)
{
    QGraphicsItemGroup* group = new QGraphicsItemGroup;

    QGraphicsRectItem* rect = new QGraphicsRectItem(0, 0, 1.0*WORLD_SCALE, .25*WORLD_SCALE);
    rect->setBrush(b);
    rect->setPen(p);
    rect->setRotation(45);
    group->addToGroup(rect);

    rect = new QGraphicsRectItem(0, 0, .25*WORLD_SCALE, 1*WORLD_SCALE);
    rect->setBrush(b);
    rect->setPen(p);
    rect->setRotation(45);
    group->addToGroup(rect);

    group->setPos(pointx*WORLD_SCALE, pointy*WORLD_SCALE);
    group->setRotation(angle);

    return group;
}
