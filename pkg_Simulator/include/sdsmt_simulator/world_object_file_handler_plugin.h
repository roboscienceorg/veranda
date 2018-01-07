#ifndef WORLD_OBJECT_FILE_HANDLER_PLUGIN_H
#define WORLD_OBJECT_FILE_HANDLER_PLUGIN_H

#include "world_object_loader_if.h"
#include "world_object_saver_if.h"

#include <QVector>

class WorldObjectFileHandler_Plugin_If
{
public:
    virtual ~WorldObjectFileHandler_Plugin_If(){}

    virtual QVector<WorldObjectLoader_If*> getLoaders() = 0;
    virtual QVector<WorldObjectSaver_If*> getSavers() = 0;
};

Q_DECLARE_INTERFACE(WorldObjectFileHandler_Plugin_If, "org.sdsmt.sim.2d.fileHandler")


#endif // WORLD_OBJECT_FILE_HANDLER_PLUGIN_H
