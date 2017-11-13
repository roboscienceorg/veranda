#include "basic_viewer.h"

#include <QPainter>
#include <QDebug>
#include <QVBoxLayout>
#include <QMouseEvent>
#include <QMessageBox>

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
    _viewer = new QGraphicsView(_scene, this);
    _viewer->setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding));

    _children->addWidget(_viewer);
    setLayout(_children);

    setWorldBounds(-30, 30, -30, 30);
}

QGraphicsItem* BasicViewer::drawb2Shape(b2Shape* s, QGraphicsItem* itemParent)
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
                qpoly.append(QPointF(vert->x*WORLD_SCALE, -vert->y*WORLD_SCALE));

            newShape = new QGraphicsPolygonItem(qpoly, itemParent);
        }
    }
    return newShape;
}

QGraphicsItem* BasicViewer::drawModel(Model* m, QGraphicsItem* parent)
{
    QGraphicsItemGroup* baseItem = new QGraphicsItemGroup(parent);

    //Draw all shapes as children of the base item
    for(b2Shape* s : m->shapes())
        baseItem->addToGroup(drawb2Shape(s));

    //Draw all children models, making them children shape objects
    //to transform to relative coordinates
    for(Model* m_sub : m->children())
        drawModel(m_sub, baseItem);

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

    for(Model* m : objects)
    {
        QGraphicsItem* graphic = drawModel(m);
        _shapes[m] = graphic;

        //If the model or one of its submodels changes, redraw the whole thing
        connect(m, &Model::modelChanged, this, &BasicViewer::modelChanged);
        connect(m, &Model::childModelChanged, this, &BasicViewer::modelChanged);
        connect(m, &Model::childTransformChanged, this, &BasicViewer::modelChanged);

        //If the base model moves, update the transform
        connect(m, &Model::transformChanged, this, &BasicViewer::modelMoved);

        _scene->addItem(graphic);
    }
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
}

void BasicViewer::modelChanged(Model *m)
{
    _scene->removeItem(_shapes[m]);

    QGraphicsItem* graphic = drawModel(m);
    _shapes[m] = graphic;

    _scene->addItem(graphic);
}

//World View Clicked
void BasicViewer::mousePressEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
       QMessageBox *msgBox ;
       msgBox = new QMessageBox();
       msgBox->setWindowTitle("Hello");
       msgBox->setText("You Clicked Left Mouse Button");
       msgBox->show();
       mouseClickPosition = event->pos();
    }
}

void BasicViewer::resizeEvent(QResizeEvent *event)
{
    rescale();
}

void BasicViewer::rescale()
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

    rescale();
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
            _shapes.remove(m);
        }
        _models.remove(id);
    }
}

//The object identified by object_id is in the world, but should not be drawn
//for users not wanting to see their sensors drawn
void BasicViewer::objectDisabled(object_id id)
{

}

//The object identified by object_id is in the world and should be drawn
void BasicViewer::objectEnabled(object_id id)
{

}

//The object identified by object_id has been selcted
//maybe we draw a selection box around it?
void BasicViewer::objectSelected(object_id id)
{

}
