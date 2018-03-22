#ifndef SCREEN_MODEL_H
#define SCREEN_MODEL_H

#include <QObject>
#include <QVector>
#include <QDebug>

#include <Box2D/Box2D.h>

#include "dllapi.h"

class SDSMT_SIMULATOR_API Model : public QObject
{
    Q_OBJECT

    QVector<Model*> _children;
    QVector<b2Shape*> _shapes;

    double _x=0, _y=0, _theta=0;

signals:
    void modelChanged(Model*);
    void transformChanged(Model*, double, double, double);
    void childModelChanged(Model*, Model*);
    void childTransformChanged(Model*, Model*);

private slots:
    void _childChildModelChanged(Model*, Model* childChild)
    {
        emit childModelChanged(this, childChild);
    }

    void _childChildTransformChanged(Model*, Model* childChild)
    {
        emit childTransformChanged(this, childChild);
    }

    void _childModelChanged(Model* child)
    {
        emit childModelChanged(this, child);
    }

    void _childTransformChanged(Model* child)
    {
        emit childTransformChanged(this, child);
    }

public:
    Model(QVector<Model*> children, QVector<b2Shape*> shapes = {}, QObject* parent = nullptr) : QObject(parent)
    {
        addChildren(children);
        addShapes(shapes);
    }

    Model(QVector<b2Shape*> shapes = {}, QObject* parent = nullptr) : QObject(parent)
    {
        addShapes(shapes);
    }

    ~Model()
    {
    }

    const QVector<b2Shape*>& shapes(){ return _shapes; }
    const QVector<Model*>& children(){ return _children; }

    void forceDraw()
    {
        modelChanged(this);
    }

    void setTransform(const double& x, const double& y, const double& theta)
    {
        double dx = x-_x, dy = y-_y, dt = theta-_theta;
        _x = x;
        _y = y;
        _theta = theta;

        //qDebug() << "Model adjusted by" << _x <<" " << _y << " " << _theta;

        //if(abs(dx) > 0.001 || abs(dy) > 0.001 || abs(dt) > 0.001)
            transformChanged(this, dx, dy, dt);
    }

    void getTransform(double& x, double& y, double& theta)
    {
        x = _x;
        y = _y;
        theta = _theta;
    }

    void addChildren(QVector<Model*> newChildren)
    {
        if(newChildren.size())
        {
            for(Model* s : newChildren)
            {
                connect(s, &Model::modelChanged, this, &Model::_childModelChanged);
                connect(s, &Model::childModelChanged, this, &Model::_childChildModelChanged);

                connect(s, &Model::transformChanged, this, &Model::_childTransformChanged);
                connect(s, &Model::childTransformChanged, this, &Model::_childChildTransformChanged);
                _children.push_back(s);
            }
            modelChanged(this);
        }
    }

    void removeChildren(QVector<Model*> oldChildren)
    {
        if(oldChildren.size())
        {
            for(Model* s : oldChildren)
            {
                _children.removeAll(s);
                disconnect(s, 0, this, 0);
            }
            modelChanged(this);
        }
    }

    void addShapes(QVector<b2Shape*> newShapes)
    {
        if(newShapes.size())
        {
            for(b2Shape* s : newShapes)
                _shapes.push_back(s);
            modelChanged(this);
        }
    }

    void removeShapes(QVector<b2Shape*> oldShapes)
    {
        if(oldShapes.size())
        {
            for(b2Shape* s : oldShapes)
                _shapes.removeAll(s);

            modelChanged(this);
        }
    }
};

#endif // SCREEN_MODEL_H
