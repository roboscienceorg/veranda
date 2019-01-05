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
 * \brief Plugin interface for load/save objects to work with single objects in JSON files
 */
class Json_Object_Plugin : public QObject, public WorldObjectFileHandler_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.roboscience.veranda.fileHandlers.jsonObject")
    Q_INTERFACES(WorldObjectFileHandler_Plugin_If)

public:
    /*!
     * \brief Gets the file loader types provided by the plugin
     * \return A single loader type that can handle JSON files with a single object in them
     */
    virtual QVector<WorldObjectLoader_If*> getLoaders();

    /*!
     * \brief Gets the file saver types provided by the plugin
     * \return A single saver type that can create JSON files with a single object in them
     */
    virtual QVector<WorldObjectSaver_If*> getSavers();
};
