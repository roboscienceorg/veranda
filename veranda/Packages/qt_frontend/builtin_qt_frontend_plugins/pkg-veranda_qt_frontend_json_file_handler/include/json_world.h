//! \file
#pragma once

#include "json_common.h"

/*!
 * \brief World loader which knows how read json files from this plugin
 */
class JsonWorldLoader : public WorldLoader_If
{
    /*!
     * \brief Returns the file types this loader can handle
     * \return "Json file (*.json)"
     */
    virtual QVector<QString> fileExts() { return QVector<QString>{"Json file (*.json)"}; }

    /*!
     * \brief Checks if a file can be loaded by this loader
     * \todo Write this function
     * \param[in] filePath Path to the file to check
     * \param[in] plugins Map of all plugins by their IID
     * \return true if the file can be loaded by this loader
     */
    virtual bool canLoadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If*> plugins){ return true; }

    /*!
     * \brief Stub function; no user input is required for this loader
     * \param[in] filePath Path to the file to check
     * \param[in] plugins Map of all plugins by their IID
     */
    virtual void getUserOptions(QString /*filePath*/, QMap<QString, WorldObjectComponent_Factory_If*> /*plugins*/){}

    /*!
     * \brief Loads an JSON file and converts it to 0 or more WorldObjects
     * \param[in] filePath Path to the file to check
     * \param[in] plugins Map of all plugins by their IID
     * \return 0 or more WorldObjects represented by the JSON objects in the file
     */
    virtual QVector<WorldObject*> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If *> plugins);
};

/*!
 * \brief World saver which creates json files
 */
class JsonWorldSaver : public WorldSaver_If
{
    /*!
     * \brief Returns the file types this saver can create
     * \return "Json file (*.json)"
     */
    virtual QVector<QString> fileExts() { return QVector<QString>{"Json file (*.json)"}; }

    /*!
     * \brief Creates a JSON file representing a group of WorldObjects
     * \param[in] filePath Path to the file to create
     * \param[in] objects The WorldObjects to save
     */
    virtual void saveFile(QString filePath, QVector<WorldObject*> objects);
};
