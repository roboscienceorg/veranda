//! \file
#pragma once

#include <QString>
#include <QVector>
#include "veranda_core/interfaces/world_object_component_factory_if.h"
#include "veranda_core/world_object.h"

/*!
 * \brief Loads single world objects from files
 * Knows how to parse a specific file or set of file types in order to retrieve
 * information about a single world object
 */
class WorldObjectLoader_If
{
public:
    //! Virtual destructor
    ~WorldObjectLoader_If(){}

    /*!
     * \brief Returns the types of files this loader can handle
     * Should follow the QFileDialog format for listing extension types --
     * "Image Files (*.jpg *.bmp *.png)"
     * "Json Files (*.json)"
     * \return QVector<QString> One or more file extension strings
     */
    virtual QVector<QString> fileExts() = 0;

    /*!
     * \brief Checks if the loader can load a specific file
     * The loader should read enough of the file to determine if it is the correct type
     * for the loader to parse. The loader should do more than just check the file extension,
     * as multiple loaders may exist for any given extension
     * \param[in] filePath Absolute path to the file to check
     * \param[in] factories Map of the available component factories in the system
     * \return True if the file can be loaded by the loader
     */
    virtual bool canLoadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If*> factories) = 0;

    /*!
     * \brief Gives the loader a chance to query the user for options
     * When loading a file, if the loader needs user input, it should be acquired here. Within this
     * function, the loader is allowed to display a dialog box to the user to get information about
     * how to load a specific file. This call is immediately followed by loadFile() with the same file
     * path.
     * \param[in] filePath Absolute path to the file to check
     * \param[in] Factories Map of the available factories in the system
     */
    virtual void getUserOptions(QString filePath, QMap<QString, WorldObjectComponent_Factory_If*> factories){}

    /*!
     * \brief Load a file and return the world object stored in it
     * The loader should read the file and use the factories list to return a single world object
     * within the file. If user settings are required, they should be gotten during getUserOptions().
     * This function may be run in a separate thread to prevent the UI from blocking; for this reason, the function
     * should not display any sort of UI or dialog box
     * \param[in] filePath Absolute path to the file to check
     * \param[in] factories Map of the available component factories in the system
     * \return A newly allocated WorldObject* built from the file
     */
    virtual WorldObject* loadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If*> factories) = 0;
};

/*!
 * \brief Loads groups of world objects from files
 * Knows how to parse a specific file or set of file types in order to retrieve
 * information about a group of world objects
 */
class WorldLoader_If
{
public:
    //! Virtual destructor
    ~WorldLoader_If(){}

    /*!
     * \brief Returns the types of files this loader can handle
     * Should follow the QFileDialog format for listing extension types --
     * "Image Files (*.jpg *.bmp *.png)"
     * "Json Files (*.json)"
     * \return QVector<QString> One or more file extension strings
     */
    virtual QVector<QString> fileExts() = 0;

    /*!
     * \brief Checks if the loader can load a specific file
     * The loader should read enough of the file to determine if it is the correct type
     * for the loader to parse. The loader should do more than just check the file extension,
     * as multiple loaders may exist for any given extension
     * \param[in] filePath Absolute path to the file to check
     * \param[in] factories Map of the available factories in the system
     * \return True if the file can be loaded by the loader
     */
    virtual bool canLoadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If*> factories) = 0;

    /*!
     * \brief Gives the loader a chance to query the user for options
     * When loading a file, if the loader needs user input, it should be acquired here. Within this
     * function, the loader is allowed to display a dialog box to the user to get information about
     * how to load a specific file. This call is immediately followed by loadFile() with the same file
     * path.
     * \param[in] filePath Absolute path to the file to check
     * \param[in] factories Map of the available factories in the system
     */
    virtual void getUserOptions(QString filePath, QMap<QString, WorldObjectComponent_Factory_If*> factories){}

    /*!
     * \brief Load a file and return the world objects stored in it
     * The loader should read the file and use the factories list to return a group of world objects
     * within the file. If user settings are required, they should be gotten during getUserOptions().
     * This function may be run in a separate thread to prevent the UI from blocking; for this reason, the function
     * should not display any sort of UI or dialog box
     * \param[in] filePath Absolute path to the file to check
     * \param[in] factories Map of the available factories in the system
     * \return A vector of newly allocated WorldObject* built from the file
     */
    virtual QVector<WorldObject*> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If*> factories) = 0;
};
