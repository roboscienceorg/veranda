#ifndef MAP_LOADER_IF_H
#define MAP_LOADER_IF_H

#include <QString>
#include <QVector>

#include <Box2D/Box2D.h>

class MapLoader_If
{
public:
    virtual QVector<b2Shape> loadMapFile(QString file) = 0;
}

#endif // MAP_LOADER_IF_H
