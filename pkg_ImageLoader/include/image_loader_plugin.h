#ifndef DEFAULT_ROBOT_LOADER_PLUGIN_H
#define DEFAULT_ROBOT_LOADER_PLUGIN_H

#include <QObject>

//This should not be fully defined relative; it should be just
//  <sdsmt_simulator/world_object_file_handler_plugin.h>
//however, a bug? in MSVC prevents the Qt MOC from resolving interfaces
//if the path isn't relative like this and it doesn't seem to work to
//do conditional compilation so this would be used only on windows
#include "../../../install/include/sdsmt_simulator/world_object_file_handler_plugin.h"

class Image_Loader_Plugin : public QObject, public WorldObjectFileHandler_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.sdsmt.sim.2d.fileHandlers.imageLoader")
    Q_INTERFACES(WorldObjectFileHandler_Plugin_If)

public:
    Image_Loader_Plugin();

    virtual QVector<WorldObjectLoader_If*> getLoaders();
    virtual QVector<WorldObjectSaver_If*> getSavers(){ return QVector<WorldObjectSaver_If*>{}; }
};

#endif
