#include "basic_viewer.h"

#include <QPainter>
#include <QDebug>
#include <QVBoxLayout>
#include <QMouseEvent>
#include <QMessageBox>

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
}

//Something new was added to the world view
//It needs to get added to the graphics scene and indexed
//When the model updates, (transformChanged) the graphics shape
//needs to be moved within the scene
void BasicViewer::modelAddedToScreen(ScreenModel_If* model, model_id id)
{
    //set the id to a model
    _models[id] = model;

    double x, y, t;
    model->getTransform(x, y, t);

    //b2Shape types are what comes from physics engine
    for(b2Shape* s : model->getModel())
    {
        QGraphicsItem* newShape = nullptr;
        switch(s->m_type)
        {
            case b2Shape::Type::e_circle:
            {
                b2CircleShape* circle = static_cast<b2CircleShape*>(s);
                double r = circle->m_radius;
                newShape = _scene->addEllipse(circle->m_p.x-r * WORLD_SCALE, -(circle->m_p.y+r) * WORLD_SCALE,
                                            circle->m_radius*2 * WORLD_SCALE, circle->m_radius*2 * WORLD_SCALE);

            }
            break;
            case b2Shape::Type::e_edge:
            {
                b2EdgeShape* edge = static_cast<b2EdgeShape*>(s);
                newShape = _scene->addLine(edge->m_vertex1.x * WORLD_SCALE, -edge->m_vertex1.y * WORLD_SCALE,
                                           edge->m_vertex2.x * WORLD_SCALE, -edge->m_vertex2.y * WORLD_SCALE);
            }
            break;
        }
        if(newShape)
        {
            _shapes[model].push_back(newShape);
            newShape->moveBy(x * WORLD_SCALE, -y * WORLD_SCALE);
            newShape->setRotation(-t);
        }
    }
    connect(model, &ScreenModel_If::transformChanged, this, &BasicViewer::modelMoved);

}


//Updates a graphics scene object to have a new
//location
void BasicViewer::modelMoved(ScreenModel_If *m, double dx, double dy, double dt)
{
    double x, y, t;
    m->getTransform(x, y, t);
    x *= WORLD_SCALE;
    y *= WORLD_SCALE;
    dx *= WORLD_SCALE;
    dy *= WORLD_SCALE;

    for(QGraphicsItem* i : _shapes[m])
    {
        i->moveBy(dx, -dy);
        i->setRotation(-t);
    }
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

//for after the MVP 
//The model identified by model_id is no longer on the world
void BasicViewer::modelRemovedFromScreen(model_id id)
{
   //

}

//The model identified by model_id is in the world, but should not be drawn
//for users not wanting to see their sensors drawn
void BasicViewer::modelDisabled(model_id id)
{

}

//The model identified by model_id is in the world and should be drawn
void BasicViewer::modelEnabled(model_id id)
{

}

//The model identified by model_id has been selcted
//maybe we draw a selection box around it?
void BasicViewer::modelSelected(model_id id)
{

}
