//! \file
#pragma once

#include <QObject>

#include <veranda_qt_frontend/file_handler_plugin.h>

/*!
 * \brief Qt Plugin interface for the image reader
 */
class Image_Loader_Plugin : public QObject, public WorldFileHandler_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.roboscience.veranda.fileHandlers.imageLoader")
    Q_INTERFACES(WorldFileHandler_Plugin_If)

public:
    /*!
     * \brief Returns an instance of the Image Loader
     * \return A vector containing 1 instance of the Image Loader
     */
    virtual QVector<WorldLoader_If*> getLoaders();

    /*!
     * \brief The plugin does not provide a way to save
     * \return An empty vector
     */
    virtual QVector<WorldSaver_If*> getSavers(){ return QVector<WorldSaver_If*>{}; }
};
