#ifndef BASIC_MAPLOADER_H
#define BASIC_MAPLOADER_H

#include "interfaces/map_loader_if.h"
#include "map.h"
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonValue>
#include <QIODevice>
#include <QFile>

class BasicMapLoader : public MapLoader_If
{
public:
    BasicMapLoader(){}
    ~BasicMapLoader(){}

    virtual Map* loadMapObject(QJsonObject world) override;
    virtual Map* loadMapFile(QString filename) override;
};

#endif // BASIC_MAPLOADER_H
