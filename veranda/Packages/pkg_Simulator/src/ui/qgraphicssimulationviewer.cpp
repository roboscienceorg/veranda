#include "ui/qgraphicssimulationviewer.h"
#include "ui_qgraphicssimulationviewer.h"

//ScreenModel_if - found in a header file

//Constructor
//Sets up widget with subwidget to view a graphics scene
//makes the
QGraphicsSimulationViewer::QGraphicsSimulationViewer(QWidget *parent) :
    Simulator_Visual_If(parent),
    ui(new Ui::qgraphicssimulationviewer)
{
    ui->setupUi(this);
    _viewer = ui->view;

    //_viewer->setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding));
    _viewer->setMouseTracking(true);

    connect(_viewer, &CustomGraphicsView::mouseMoved, this, &QGraphicsSimulationViewer::viewMouseMove);
    connect(_viewer, &CustomGraphicsView::mousePress, this, &QGraphicsSimulationViewer::viewMousePress);
    connect(_viewer, &CustomGraphicsView::mouseRelease, this, &QGraphicsSimulationViewer::viewMouseRelease);

    _translater = _makeTranslater();
    _rotater = _makeRotater();

    _tools = new QGraphicsItemGroup;
    _tools->addToGroup(_translater);
    _tools->addToGroup(_rotater);
    _translater->moveBy(-_rotater->boundingRect().width()*1.5, 0);
    setNavigationEnabled(true);

    _resetScene();
}

QGraphicsSimulationViewer::~QGraphicsSimulationViewer()
{
    delete ui;
}

void QGraphicsSimulationViewer::_resetScene()
{
    if(_scene)
    {
        if(_tools->scene())
            _scene->removeItem(_tools);

        _scene->deleteLater();
        _scene = nullptr;
    }

    _scene = new QGraphicsScene(this);
    _viewer->setScene(_scene);
}

void QGraphicsSimulationViewer::setNavigationEnabled(bool allowed)
{
    if(allowed)
    {
        connect(_viewer, &CustomGraphicsView::zoomTick, this, &QGraphicsSimulationViewer::viewZoom);
        connect(_viewer, &CustomGraphicsView::screenShift, this, &QGraphicsSimulationViewer::viewShift);
    }
    else
    {
        disconnect(_viewer, &CustomGraphicsView::zoomTick, this, &QGraphicsSimulationViewer::viewZoom);
        disconnect(_viewer, &CustomGraphicsView::screenShift, this, &QGraphicsSimulationViewer::viewShift);
    }

    ui->scroll_horizontal->setVisible(allowed);
    ui->scroll_vertical->setVisible(allowed);
    ui->button_zoomIn->setVisible(allowed);
    ui->button_zoomOut->setVisible(allowed);
}

/*!
 * The supported b2Shape types are
 * * Circle
 * * Edge
 * * Polygon
 *
 * Drawing any of these results in a single QGrahicsItem with no children
 */
QGraphicsItem* QGraphicsSimulationViewer::_drawb2Shape(b2Shape* s, QGraphicsItem* itemParent)
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

QGraphicsItemGroup* QGraphicsSimulationViewer::_drawModel(Model* m)
{
    QGraphicsItemGroup* baseItem = new QGraphicsItemGroup();

    //Draw all shapes as children of the base item
    for(b2Shape* s : m->shapes())
        baseItem->addToGroup(_drawb2Shape(s));

    //Set location of model
    double x, y, t;
    m->getTransform(x, y, t);

    //qDebug() << "Drew model at " << x << y << t << "from parent";

    baseItem->moveBy(x * WORLD_SCALE, -y*WORLD_SCALE);
    baseItem->setRotation(-t);

    return baseItem;
}

/*!
 * Something new was added to the world view
 * It needs to get added to the graphics scene and indexed
 * When the model updates, (transformChanged) the graphics shape
 * needs to be moved within the scene
 */
void QGraphicsSimulationViewer::objectAddedToScreen(QVector<Model*> objects, object_id id)
{
    _models[id] = objects;

    _drawLevels[id] = Solid;

    //qDebug() << "Add object " << id;

    QGraphicsItemGroup* group = new QGraphicsItemGroup();
    for(Model* m : objects)
    {
        //qDebug() << "Draw top level model";
        QGraphicsItem* graphic = addModel(m, id);

        group->addToGroup(graphic);
        _updateColoring(m);
    }
    _scene->addItem(group);

    _shapeToObject[group] = id;
    _topShapes[id] = group;

    if(_models.size() == 1)
    {
        _fitInView(_scene->sceneRect());
    }
}

/*!
 * When adding a new model, we create a number of ways to map to and from it
 * * Map the model to the QGraphicItem
 * * Map the QGraphicItem to the object id of the model
 * * Map the model to its object id
 * * Mapthe model to a list of its child models
 *
 * This method also connects the signals from the model which
 * indicate that the model moved or its shapes or children were changed
 */
QGraphicsItem* QGraphicsSimulationViewer::addModel(Model *m, object_id id)
{
    //qDebug() << "Add model " << m << " with " << m->children().size() << " children";
    QGraphicsItemGroup* graphic = _drawModel(m);

    _shapes[m] = graphic;
    _shapeToObject[graphic] = id;
    _modelToObject[m] = id;
    _modelChildren[m] = m->children();

    //If the model or one of its submodels changes, redraw the whole thing
    connect(m, &Model::modelChanged, this, &QGraphicsSimulationViewer::modelChanged);

    //If the base model moves, update the transform
    connect(m, &Model::transformChanged, this, &QGraphicsSimulationViewer::modelMoved);

    //If the drawhint for the model changes, update it's pen and brush
    connect(m, &Model::hintChanged, this, &QGraphicsSimulationViewer::modelHinted);

    for(Model* child : m->children())
    {
        //qDebug() << "Draw child of " << m;
        QGraphicsItem* cGraph = addModel(child, id);
        cGraph->setParentItem(graphic);
    }

    return graphic;
}

//! When a model draw hint changes we update it and its children
void QGraphicsSimulationViewer::modelHinted(Model *m)
{
    _updateColoring(m);
}

/*!
 * When a model moves, we find the QGraphicsItem that represents it
 * and move that the same amount
 */
void QGraphicsSimulationViewer::modelMoved(Model* m, const double &dx, const double &dy, const double &dt)
{
    double x, y, t;
    m->getTransform(x, y, t);

    x *= WORLD_SCALE;
    y *= WORLD_SCALE;

    _shapes[m]->moveBy(dx * WORLD_SCALE, -dy * WORLD_SCALE);
    _shapes[m]->setRotation(-t);

    if(_modelToObject[m] == _currSelection)
        _placeTools();
}

/*!
 * When a model changes, we destroy the QGraphicsItem that
 * was representing it and rebuild it
 */
void QGraphicsSimulationViewer::modelChanged(Model *m)
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

    _updateColoring(m);

    if(parent == nullptr)
        _scene->addItem(replace);
}

/*!
 * When the left mouse button is pressed, start tracking
 * the mouse if the user selected one of the click-drag tools.
 *
 * If the user did not click one of the click-drag tools, we check
 * if they clicked on one of the drawn models, and if so signal that
 * it should be the selected model
 */
void QGraphicsSimulationViewer::viewMousePress(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        QPointF hit = event->localPos();

        bool startDrag = _toolsEnabled && _translater->contains(
                    _translater->sceneTransform().inverted().map(
                        _viewer->mapToScene(
                            _viewer->mapFromGlobal(
                                event->globalPos()))));

        bool startRotate = _toolsEnabled && _rotater->contains(
                    _rotater->sceneTransform().inverted().map(
                        _viewer->mapToScene(
                            _viewer->mapFromGlobal(
                                event->globalPos()))));

        if(!_draggingTranslate && startDrag)
        {
            //qDebug() << "Start dragging";
            _draggingRotate = false;
            _draggingTranslate = true;
            _dragStart = _viewer->mapToScene(
                        _viewer->mapFromGlobal(
                            event->globalPos()));
        }
        else if(!_draggingRotate && startRotate)
        {
            //qDebug() << "Start rotating";
            _draggingTranslate = false;
            _draggingRotate = true;
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

/*!
 * If the user is click-dragging one of the tools, then
 * emit signals that the object which is currently selected
 * is being moved or rotate.
 */
void QGraphicsSimulationViewer::viewMouseMove(QMouseEvent* event)
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
    else if(_draggingRotate)
    {
        QPointF newLocal = _viewer->mapToScene(
                    _viewer->mapFromGlobal(
                        event->globalPos()));

        QRectF bound = _topShapes[_currSelection]->childrenBoundingRect();
        QPointF center(bound.left() + bound.width()/2.0, bound.top() + bound.height()/2.0);
        QPointF v1 = _dragStart - center;
        QPointF v2 = newLocal - center;
        double delta = (std::atan2(v2.y(),v2.x()) - std::atan2(v1.y(),v1.x())) * RAD2DEG;

        //Filter out jumpiness
        //There's probably a more technically correct way to do this
        if(std::abs(delta) < 90)
        {
            userDragRotateObject(_currSelection, -delta);
        }
        _dragStart = newLocal;
    }
}

void QGraphicsSimulationViewer::setToolsEnabled(bool enabled)
{
    _toolsEnabled = enabled;
    if(enabled)
    {
        if(_topShapes.contains(_currSelection))
        {
            _placeTools();
            _scene->addItem(_tools);
        }
    }
    else
    {
        _scene->removeItem(_tools);
    }
}

void QGraphicsSimulationViewer::viewMouseRelease(QMouseEvent *event)
{
    _draggingTranslate = false;
    _draggingRotate = false;
}

void QGraphicsSimulationViewer::resizeEvent(QResizeEvent *event)
{
    QRectF viewport = _viewer->geometry();
    double dx, dy;
    if((dx = (viewport.width() - _targetView.width())) > 0)
    {
        _targetView.translate(dx/2, 0);
        _targetView.setWidth(_targetView.width()-dx);
    }

    if((dy = (viewport.height() - _targetView.height())) > 0)
    {
        _targetView.translate(dy/2, 0);
        _targetView.setHeight(_targetView.height()-dy);
    }

    _fitInView(_targetView);
}


/*!
 * It appears that QGraphicsView::fitInView is broken in
 * Qt 5.5. This should be an acceptable alternative. We manually
 * calculate how much of the view should be visible based on the
 * canvas size and the scene size and rescale the viewport
 */
void QGraphicsSimulationViewer::_fitInView(const QRectF &targetView)
{
    QRectF viewSize = _viewer->geometry();

    double scale = std::min(viewSize.width()/targetView.width(), viewSize.height()/targetView.height());

    QTransform matrix;
    matrix.scale(scale,
                 scale);
    _viewer->setTransform(matrix);

    _viewer->centerOn(targetView.center());
    _targetView = targetView;
}

void QGraphicsSimulationViewer::viewShift(const int& x, const int& y)
{
    double pctx = x*0.1;
    double pcty = y*0.1;

    _targetView.translate(_targetView.width()*pctx, _targetView.height()*pcty);

    _fitInView(_targetView);
}

void QGraphicsSimulationViewer::viewZoom(const int& z)
{
    double dx = _targetView.width() * 0.1 * z;
    double dy = _targetView.height() * 0.1 * z;

    _targetView.translate(dx/2, dy/2);
    _targetView.setWidth(_targetView.width()+dx);
    _targetView.setHeight(_targetView.height()+dy);

    _fitInView(_targetView);
}

//for after the MVP
//The object identified by object_id is no longer on the world
void QGraphicsSimulationViewer::objectRemovedFromScreen(object_id id)
{
    //qDebug() << "Removing object" << id << "?";
    if(_models.contains(id))
    {
        //qDebug() << "Removing";
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

    if(_currSelection == id)
        nothingSelected();

    //If last item is removed, reset the scene
    //entirely to shrink the auto view rect
    if(_models.empty())
        _resetScene();
}

void QGraphicsSimulationViewer::removeModel(Model *m)
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
void QGraphicsSimulationViewer::objectDrawLevelSet(object_id id, DrawLevel level)
{
    _drawLevels[id] = level;
    for(Model* m : _models[id])
        _updateColoring(m);
}

//The object identified by object_id has been selcted
//maybe we draw a selection box around it?
void QGraphicsSimulationViewer::objectSelected(object_id id)
{
    if(id != _currSelection)
    {
        nothingSelected();

        if(_models.contains(id))
        {
            _currSelection = id;

            for(Model* m : _models[_currSelection])
                _updateColoring(m);

            if(_toolsEnabled)
            {
                _placeTools();
                _scene->addItem(_tools);
            }
        }
    }
}

void QGraphicsSimulationViewer::_placeTools()
{
    if(_topShapes.contains(_currSelection) && _tools)
    {
        QRectF bound = _topShapes[_currSelection]->childrenBoundingRect();
        double rightEdge = bound.x() + bound.width(), bottomEdge = bound.y() + bound.height();

        QRectF toolBound = _tools->childrenBoundingRect();
        _tools->setPos(rightEdge /*+ toolBound.width()*0.5*/, bottomEdge + toolBound.height());
    }
}

//No objects are selected, draw without highlights
void QGraphicsSimulationViewer::nothingSelected()
{
    if(_currSelection != 0)
    {
        auto prevSelection = _currSelection;
        _currSelection = 0;

        if(_models.contains(prevSelection))
            for(Model* m : _models[prevSelection])
                _updateColoring(m);
    }

    if(_tools->scene())
        _scene->removeItem(_tools);
}

uint8_t QGraphicsSimulationViewer::_getAlpha(Model* m)
{
    object_id modelObject = _modelToObject[m];
    DrawLevel level = _drawLevels[modelObject];

    switch(level) {
        case Solid: return 255;
        case Transparent: return 127;
        case Off: return 0;
    }

    return 255;
}

/*!
 * The QObjectItems drawn by this widget are one of three types:
 * * QAbstractGraphicsShapeItem
 * * QGraphicsLineItem
 * * QGraphicsItemGroup
 *
 * In the first two cases, the color can just be set. After possibly setting
 * the color, the method recurses on the children GraphicsItems
 */
void QGraphicsSimulationViewer::_updateColoring(Model* m)
{
    // Bookkeeping to get data about model
    object_id modelObject = _modelToObject[m];
    uint8_t alpha = _getAlpha(m);

    //Record of colors to use
    QColor penColor(m->getDrawHint().outlineColor);
    QColor brushColor(m->getDrawHint().fillColor);

    //Record of styles to use
    QPen pen(m->getDrawHint().outlineStyle);
    QBrush brush(m->getDrawHint().fillStyle);

    //If inheriting draw rules and parent is known, use its pen and brush
    auto it = _modelPensBrushes.begin();
    if(m->getDrawHint().inherit && m->getParent() &&
      (it = _modelPensBrushes.find(m->getParent())) != _modelPensBrushes.end())
    {
        penColor = it.value().first.color();
        brushColor = it.value().second.color();

        pen.setStyle(it.value().first.style());
        brush.setStyle(it.value().second.style());
    }

    //Get colors; override drawhint color when selected
    if(_currSelection == modelObject)
    {
        penColor = SELECTED_COLOR;
        brushColor =  SELECTED_COLOR;
    }

    //Set alpha rules regardless of other rules
    penColor.setAlpha(alpha);
    brushColor.setAlpha(alpha);

    //Merge styles and colors, and save
    pen.setColor(penColor);
    brush.setColor(brushColor);

    _modelPensBrushes[m] = {pen, brush};

    //Grab shapes onscreen assocated
    QGraphicsItemGroup* itemGroup = _shapes[m];

    //We know that all models are drawn as an itemgroup
    //with various single shapes inside (and then also the child models' groups)
    //For each of those non-group shapes, update the coloring
    for(QGraphicsItem* child : itemGroup->childItems())
    {
        //Using dynamic cast because the qgraphics item cast always returns nullptr with
        //abstract classes
        QAbstractGraphicsShapeItem* asShape = dynamic_cast<QAbstractGraphicsShapeItem*>(child);

        if(asShape)
        {
            asShape->setPen(pen);
            asShape->setBrush(brush);
        }
        else
        {
            //QGraphicsLine is not a QAbstractGraphicsShapeItem so it needs to be a special check
            QGraphicsLineItem* asLine = dynamic_cast<QGraphicsLineItem*>(child);
            if(asLine)
            {
                asLine->setPen(pen);
            }
        }
    }

    //Update all child colors
    //if they inherit, then they will use the pen
    //and brush determined here
    for(Model* child : m->children())
        _updateColoring(child);
}

QGraphicsItem* QGraphicsSimulationViewer::_makeTranslater()
{
    QBrush b(SELECTED_COLOR);
    QPen p(SELECTED_COLOR);

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

QGraphicsItem* QGraphicsSimulationViewer::_makeRotater()
{
    QBrush b(SELECTED_COLOR);
    QPen p(SELECTED_COLOR);

    QGraphicsItemGroup* group = new QGraphicsItemGroup;

    QGraphicsRectItem* rect = new QGraphicsRectItem(0.0, -1.0*WORLD_SCALE, 0.5*WORLD_SCALE, 1.25*WORLD_SCALE);
    rect->setBrush(b);
    rect->setPen(p);
    group->addToGroup(rect);

    rect = new QGraphicsRectItem(-1.0*WORLD_SCALE, 0.0, 1.25*WORLD_SCALE, 0.5*WORLD_SCALE);
    rect->setBrush(b);
    rect->setPen(p);

    group->addToGroup(rect);
    group->addToGroup(_makeArrow(.25, -1.3, 0, p, b));
    //group->addToGroup(_makeArrow(.25, 1.8, 180, p, b));
    //group->addToGroup(_makeArrow(1.8, .25, 90, p, b));
    group->addToGroup(_makeArrow(-1.3, .25, 270, p, b));

    return group;
}

QGraphicsItem* QGraphicsSimulationViewer::_makeArrow(double pointx, double pointy, double angle, QPen p, QBrush b)
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
