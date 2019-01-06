//! \file
#pragma once

#include "json_common.h"

/*!
 * \brief WorldObject loader which knows how read json files from this plugin
 */
class JsonObjectLoader : public WorldObjectLoader_If
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
    virtual void getUserOptions(QString filePath, QMap<QString, WorldObjectComponent_Factory_If*> plugins){}

    /*!
     * \brief Loads an JSON file and converts it to a WorldObject
     * \param[in] filePath Path to the file to check
     * \param[in] plugins Map of all plugins by their IID
     * \return A single WorldObject represented by the JSON object in the file
     */
    virtual WorldObject* loadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If *> plugins);
};

/*!
 * \brief WorldObject saver which creates json files
 */
class JsonObjectSaver : public WorldObjectSaver_If
{
    /*!
     * \brief Returns the file types this saver can create
     * \return "Json file (*.json)"
     */
    virtual QVector<QString> fileExts() { return QVector<QString>{"Json file (*.json)"}; }

    /*!
     * \brief Creates a JSON file representing a single WorldObject
     * \param[in] filePath Path to the file to create
     * \param[in] objects The WorldObject to save
     */
    virtual void saveFile(QString filePath, WorldObject* objects);
};
