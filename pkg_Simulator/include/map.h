#ifndef MAP_H
#define MAP_H

#include <QVector>
#include <Box2D/Box2D.h>

class Map
{
    int xMin;
    int xMax;
    int yMin;
    int yMax;
    int xOrigin;
    int yOrigin;
    QVector<b2Shape *> staticBodies;
public:
    Map();
    bool setXMin(int i);
    bool setXMax(int i);
    bool setYMin(int i);
    bool setYMax(int i);
    bool setXOrigin(int i);
    bool setYOrigin(int i);
    bool setStaticBodies(QVector<b2PolygonShape *> polygons);
};

#endif // MAP_H
