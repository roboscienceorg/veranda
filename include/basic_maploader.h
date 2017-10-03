#ifndef BASIC_MAPLOADER_H
#define BASIC_MAPLOADER_H

#include "interfaces/map_loader_if.h"

class BasicMapLoader : public MapLoader_If
{
    virtual QString loadMapFile(QString file, QVector<b2Shape*>& result) override;
};

#endif // BASIC_MAPLOADER_H
