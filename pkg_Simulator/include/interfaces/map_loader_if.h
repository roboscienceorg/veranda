#ifndef MAP_LOADER_IF_H
#define MAP_LOADER_IF_H

#include <QString>
#include <QVector>

#include <Box2D/Box2D.h>

class MapLoader_If
{
public:
    virtual QString loadMapFile(QString file, QVector<b2Shape*>& result) = 0;
};

#endif // MAP_LOADER_IF_H
