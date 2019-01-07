//! \file
#pragma once

#include <QObject>

#include <veranda_qt_frontend/file_handler_plugin.h>

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
