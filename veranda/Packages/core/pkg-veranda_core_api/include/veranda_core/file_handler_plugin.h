//! \file

#pragma once
#include "veranda_core/object_loader_if.h"
#include "veranda_core/object_saver_if.h"
#include "veranda_core/dllapi.h"

#include <QVector>

/*!
 * \brief Plugin interface for file handlers of single world objects
 *
 * This interface should be used for file handling plugins that
 * load and/or save files which contain information about a single
 * world object
 */
class WorldObjectFileHandler_Plugin_If
{
public:
    //! Virtual destructor
    virtual ~WorldObjectFileHandler_Plugin_If(){}

    /*!
     * \brief Getter for the file loaders provided by the plugin
     * \return QVector<WorldObjectLoader_If*> - 0 or more object file loaders allocated on the heap
     */
    virtual QVector<WorldObjectLoader_If*> getLoaders() = 0;

    /*!
     * \brief Getter for the file loaders provided by the plugin
     * \return QVector<WorldObjectSaver_If*> - 0 or more object file savers allocated on the heap
     */
    virtual QVector<WorldObjectSaver_If*> getSavers() = 0;
};

Q_DECLARE_INTERFACE(WorldObjectFileHandler_Plugin_If, "org.roboscience.veranda.objFileHandler")

/*!
 * \brief Plugin interface for file handlers of full simulations
 *
 * This interface should be used for file handling plugins that
 * load and/or save files which contain information about an entire
 * simulation (Multiple world objects)
 */
class WorldFileHandler_Plugin_If
{
public:
    //! Virtual destructor
    virtual ~WorldFileHandler_Plugin_If(){}

    /*!
     * \brief Getter for the file loaders provided by the plugin
     * \return QVector<WorldLoader_If*> - 0 or more world file loaders allocated on the heap
     */
    virtual QVector<WorldLoader_If*> getLoaders() = 0;

    /*!
     * \brief Getter for the file loaders provided by the plugin
     * \return QVector<WorldSaver_If*> - 0 or more world file savers allocated on the heap
     */
    virtual QVector<WorldSaver_If*> getSavers() = 0;
};

Q_DECLARE_INTERFACE(WorldFileHandler_Plugin_If, "org.roboscience.veranda.wrldFileHandler")
