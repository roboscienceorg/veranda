#ifndef WORLD_OBJECT_FILE_HANDLER_PLUGIN_H
#define WORLD_OBJECT_FILE_HANDLER_PLUGIN_H

#include "object_loader_if.h"
#include "object_saver_if.h"
#include "dllapi.h"

#include <QVector>

class WorldObjectFileHandler_Plugin_If
{
public:
    virtual ~WorldObjectFileHandler_Plugin_If(){}

    virtual QVector<WorldObjectLoader_If*> getLoaders() = 0;
    virtual QVector<WorldObjectSaver_If*> getSavers() = 0;
};

Q_DECLARE_INTERFACE(WorldObjectFileHandler_Plugin_If, "org.sdsmt.sim.2d.objFileHandler")

class WorldFileHandler_Plugin_If
{
public:
    virtual ~WorldFileHandler_Plugin_If(){}

    virtual QVector<WorldLoader_If*> getLoaders() = 0;
    virtual QVector<WorldSaver_If*> getSavers() = 0;
};

Q_DECLARE_INTERFACE(WorldFileHandler_Plugin_If, "org.sdsmt.sim.2d.wrldFileHandler")


#endif // WORLD_OBJECT_FILE_HANDLER_PLUGIN_H
