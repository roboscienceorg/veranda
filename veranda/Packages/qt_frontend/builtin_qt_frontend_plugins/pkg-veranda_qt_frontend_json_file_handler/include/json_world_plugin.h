//! \file
#pragma once

#include <QObject>

#include <veranda_qt_frontend/file_handler_plugin.h>

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
