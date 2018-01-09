#ifndef DEFAULT_ROBOT_LOADER_PLUGIN_H
#define DEFAULT_ROBOT_LOADER_PLUGIN_H

#include <QObject>

#include <sdsmt_simulator/world_object_file_handler_plugin.h>

class Default_Robot_Loader_Plugin : public QObject, public WorldObjectFileHandler_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.sdsmt.sim.2d.fileHandlers.defaultBot")
    Q_INTERFACES(WorldObjectFileHandler_Plugin_If)

public:
    Default_Robot_Loader_Plugin();

    virtual QVector<WorldObjectLoader_If*> getLoaders();
    virtual QVector<WorldObjectSaver_If*> getSavers(){ return QVector<WorldObjectSaver_If*>{}; }
};

#endif
