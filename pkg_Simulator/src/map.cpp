#include "map.h"
#include "sdsmt_simulator/cloneshape.h"

#include <QVector>

Map::Map(QObject *parent) : depracatedWorldObject_If(parent)
{
    model = new Model({}, {}, this);
}

Map::~Map()
{
    QVector<b2Shape*> prevShapes = model->shapes();
    model->removeShapes(prevShapes);
    delete model;
    qDeleteAll(prevShapes);
}

bool Map::setXMin(double i)
{
    xMin.set(i);

    return true;
}

bool Map::setXMax(double i)
{
    xMax.set(i);

    return true;
}

bool Map::setYMin(double i)
{
    yMin.set(i);

    return true;
}

bool Map::setYMax(double i)
{
    yMax.set(i);

    return true;
}

bool Map::setXOrigin(double i)
{
    double orig = xOrigin.get().toDouble();
    xOrigin.set(i);

    _shiftObstaclesAroundOrigin(orig, yOrigin.get().toDouble(), i, yOrigin.get().toDouble());
    return true;
}

bool Map::setYOrigin(double i)
{
    double orig = yOrigin.get().toDouble();
    yOrigin.set(i);

    _shiftObstaclesAroundOrigin(xOrigin.get().toDouble(), orig, xOrigin.get().toDouble(), i);
    return true;
}

void Map::getBounds(double& xMin_, double& yMin_, double& xMax_, double& yMax_)
{
    xMin_ = xMin.get().toDouble();
    yMin_ = yMin.get().toDouble();
    xMax_ = xMax.get().toDouble();
    yMax_ = yMax.get().toDouble();
}

void Map::_shiftObstaclesAroundOrigin(double oldX, double oldY, double newX, double newY)
{
    double xDiff = oldX - newX;
    double yDiff = oldY - newY;

    for(b2PolygonShape* p : staticBodies)
    {
        p->m_centroid = b2Vec2(p->m_centroid.x + xDiff, p->m_centroid.y + yDiff);
    }

    _buildModel();
}

void Map::_buildModel()
{
    QVector<b2Shape*> shapes;
    for(b2PolygonShape* p : staticBodies)
    {
        QVector<b2Vec2> points;
        for(b2Vec2* v = p->m_vertices; v != p->m_vertices + p->m_count; v++)
        {
            points.push_back(b2Vec2(p->m_centroid.x + v->x, p->m_centroid.y + v->y));
        }

        b2PolygonShape* adjusted = new b2PolygonShape;
        adjusted->Set(points.data(), points.size());

        shapes.push_back(adjusted);
    }

    QVector<b2Shape*> prevShapes = model->shapes();
    model->removeShapes(prevShapes);
    model->addShapes(shapes);
    qDeleteAll(prevShapes);
}

bool Map::setStaticBodies(QVector<b2PolygonShape *> polygons)
{
    staticBodies = polygons;
    _buildModel();

    return true;
}

void Map::setStaticBodies(QVector<b2Body*>& bodies)
{
    int i = 0;
    for(b2Shape* s : model->shapes())
    {
        b2FixtureDef fixDef;
        fixDef.density = 1;
        fixDef.friction = 0;
        fixDef.isSensor = false;
        fixDef.shape = s;

        bodies[i]->CreateFixture(&fixDef);
    }
}

depracatedWorldObject_If* Map::clone(QObject* newParent)
{
    Map* newMap = new Map(newParent);

    newMap->xMax.set(xMax.get());
    newMap->xMin.set(xMin.get());
    newMap->yMax.set(yMax.get());
    newMap->yMin.set(yMin.get());
    newMap->xOrigin.set(xOrigin.get());
    newMap->yOrigin.set(yOrigin.get());

    newMap->setStaticBodies(staticBodies);

    return newMap;
}
