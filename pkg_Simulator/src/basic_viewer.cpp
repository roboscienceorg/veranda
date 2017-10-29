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

    //b2Shape types are what comes from physics engine
    for(b2Shape* s : model->getModel())
    {
        switch(s->m_type)
        {
            case b2Shape::Type::e_circle:
            {
                b2CircleShape* circle = static_cast<b2CircleShape*>(s);
                if(!_shapes[model])
                {
                    _shapes[model] = _scene->addEllipse(circle->m_p.x-circle->m_radius, circle->m_p.y-circle->m_radius,
                                                        circle->m_radius*2, circle->m_radius*2);
                    connect(model, &ScreenModel_If::transformChanged, this, &BasicViewer::modelMoved);
                }
            }
            break;
        }
    }
}

//Updates a graphics scene object to have a new
//location
void BasicViewer::modelMoved(ScreenModel_If *m)
{
    double x, y, t;
    //getTransform returns the x, y positions as well as the roation
    m->getTransform(x, y, t);
    _shapes[m]->setPos(x, y);
    _shapes[m]->setRotation(t);
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
