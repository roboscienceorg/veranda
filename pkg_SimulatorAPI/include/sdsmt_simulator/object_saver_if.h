//! \file
#pragma once

#include <QString>
#include <QVector>
#include <QFile>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include "world_object.h"
#include "dllapi.h"


/*!
 * \brief Saves single world objects into files
 * Knows how to create a specific file type containing the information
 * associated with a world object.
 *
 * Generally paired with a worldObjectLoader_If for the same extension; but could
 * also be used purely to export to a different application's format
 */
class WorldObjectSaver_If
{
public:
    //! Virtual destructor
    ~WorldObjectSaver_If(){}

    /*!
     * \brief Returns the types of files this save can create
     * Should follow the QFileDialog format for listing extension types --
     * "Image Files (*.jpg *.bmp *.png)"
     * "Json Files (*.json)"
     * \return QVector<QString> One or more file extension strings
     */
    virtual QVector<QString> fileExts() = 0;

    /*!
     * \brief Saves a worldObject into a file
     * \param filePath The path to the file to save into
     * \param object The WorldObject to save
     */
    virtual void saveFile(QString filePath, WorldObject* object) = 0;

};

/*!
 * \brief Saves groups of world objects into files
 * Knows how to create a specific file type containing the information
 * associated with a world object.
 *
 * Generally paired with a worldLoader_If for the same extension; but could
 * also be used purely to export to a different application's format
 */
class WorldSaver_If
{
public:
    //! Virtual destructor
    ~WorldSaver_If(){}

    /*!
     * \brief Returns the types of files this save can create
     * Should follow the QFileDialog format for listing extension types --
     * "Image Files (*.jpg *.bmp *.png)"
     * "Json Files (*.json)"
     * \return QVector<QString> One or more file extension strings
     */
    virtual QVector<QString> fileExts() = 0;

    /*!
     * \brief Saves a group of worldObjects into a file
     * \param filePath The path to the file to save into
     * \param objects The WorldObjects to save
     */
    virtual void saveFile(QString filePath, QVector<WorldObject*> objects) = 0;
};
