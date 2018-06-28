//! \file
#pragma once

#include <QObject>

//This should not be fully defined relative; it should be just
//  <veranda/world_object_file_handler_plugin.h>
//however, a bug? in MSVC prevents the Qt MOC from resolving interfaces
//if the path isn't relative like this and it doesn't seem to work to
//do conditional compilation so this would be used only on windows
#include "../../../../../install/include/veranda/file_handler_plugin.h"

/*!
 * \brief Plugin interface for the default WorldObjects loader
 */
class Default_Robot_Loader_Plugin : public QObject, public WorldFileHandler_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.roboscience.veranda.fileHandlers.defaultBot")
    Q_INTERFACES(WorldFileHandler_Plugin_If)

public:
    /*!
     * \brief Gets the loader types provided by this plugin
     * \return An instance of the DefaultRobotLoader
     */
    virtual QVector<WorldLoader_If*> getLoaders();

    /*!
     * \brief Gets the saver types provided by this plugin
     * \return An empty vector
     */
    virtual QVector<WorldSaver_If*> getSavers(){ return QVector<WorldSaver_If*>{}; }
};
