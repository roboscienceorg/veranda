#ifndef MAP_LOADER_IF_H
#define MAP_LOADER_IF_H

#include <QString>
#include <QVector>
#include <QJsonObject>

#include <Box2D/Box2D.h>
#include "map.h"

class MapLoader_If
{
public:
    MapLoader_If(){}
    virtual ~MapLoader_If(){}

    virtual Map* loadMapObject(QJsonObject world) = 0;
    virtual Map* loadMapFile(QString filename) = 0;
};

#endif // MAP_LOADER_IF_H
