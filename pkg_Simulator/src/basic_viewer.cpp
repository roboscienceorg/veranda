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
                double r = circle->m_radius;
                qDebug() << "Circle at " << circle->m_p.x << circle->m_p.y << ":" << circle->m_radius;
                _shapes[model].push_back(_scene->addEllipse(circle->m_p.x-r, -(circle->m_p.y+r),
                                            circle->m_radius*2, circle->m_radius*2));

            }
            case b2Shape::Type::e_edge:
            {
                b2EdgeShape* edge = static_cast<b2EdgeShape*>(s);
                _shapes[model].push_back(_scene->addLine(edge->m_vertex1.x, -edge->m_vertex1.y, edge->m_vertex2.x, -edge->m_vertex2.y));
            }
            break;
        }
    }
    connect(model, &ScreenModel_If::transformChanged, this, &BasicViewer::modelMoved);

}

void BasicViewer::modelMoved(ScreenModel_If *m, double dx, double dy, double dt)
{
    for(QGraphicsItem* i : _shapes[m])
    {
        i->moveBy(dx, -dy);
        i->setRotation(i->rotation() + dt);
    }
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
