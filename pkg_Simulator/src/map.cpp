#include "map.h"
#include <QVector>

Map::Map()
{

}

bool Map::setXMin(int i)
{
    xMin = i;

    return true;
}

bool Map::setXMax(int i)
{
    xMax = i;

    return true;
}

bool Map::setYMin(int i)
{
    yMin = i;

    return true;
}

bool Map::setYMax(int i)
{
    yMax = i;

    return true;
}

bool Map::setXOrigin(int i)
{
    xOrigin = i;

    return true;
}

bool Map::setYOrigin(int i)
{
    yOrigin = i;

    return true;
}

bool Map::setStaticBodies(QVector<b2PolygonShape *> polygons)
{
    for( int i = 0; i < polygons.size(); i++)
        staticBodies.push_back(polygons[i]);

    return true;
}
