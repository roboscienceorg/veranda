//! \file
#pragma once

#include <QObject>

//This should not be fully defined relative; it should be just
//  <veranda_core/world_object_file_handler_plugin.h>
//however, a bug? in MSVC prevents the Qt MOC from resolving interfaces
//if the path isn't relative like this and it doesn't seem to work to
//do conditional compilation so this would be used only on windows
#include "../../../../../../install/include/veranda_core_api/veranda_core/file_handler_plugin.h"

/*!
 * \brief Plugin interface for load/save objects to work with multiple objects in JSON files
 */
class Json_World_Plugin : public QObject, public WorldFileHandler_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.roboscience.veranda.fileHandlers.jsonWorld")
    Q_INTERFACES(WorldFileHandler_Plugin_If)

public:
    /*!
     * \brief Gets the file loader types provided by the plugin
     * \return A single loader type that can handle JSON files with multiple objects in them
     */
    virtual QVector<WorldLoader_If*> getLoaders();

    /*!
     * \brief Gets the file saver types provided by the plugin
     * \return A single saver type that can create JSON files with multiple objects in them
     */
    virtual QVector<WorldSaver_If*> getSavers();
};
