#include "basic_viewer.h"

#include <QPainter>
#include <QDebug>
#include <QVBoxLayout>


BasicViewer::BasicViewer(QWidget *parent) : Simulator_Visual_If(parent)
{
    _children = new QVBoxLayout(this);

    _scene = new QGraphicsScene(this);
    _viewer = new QGraphicsView(_scene, this);
    _viewer->setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding));

    _children->addWidget(_viewer);
    setLayout(_children);
}

void BasicViewer::modelAddedToScreen(ScreenModel_If* model, model_id id)
{
    _models[id] = model;

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

void BasicViewer::modelMoved(ScreenModel_If *m)
{
    double x, y, t;
    m->getTransform(x, y, t);
    _shapes[m]->setPos(x, y);
    _shapes[m]->setRotation(t);
}

void BasicViewer::modelRemovedFromScreen(model_id id)
{
}

void BasicViewer::modelDisabled(model_id id)
{

}

void BasicViewer::modelEnabled(model_id id)
{

}

void BasicViewer::modelSelected(model_id id)
{

}
