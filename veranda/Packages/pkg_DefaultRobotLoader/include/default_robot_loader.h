//! \file
#pragma once

#include <QVector>
#include <QString>

#include <veranda/object_loader_if.h>

/*!
 * \brief Loader class to create a set of default robot objects for testing purposes
 */
class DefaultRobotLoader : public WorldLoader_If
{
    /*!
     * \brief Required method stub for file extensions that can be 'loaded'
     * \return Empty string
     */
    virtual QVector<QString> fileExts() { return QVector<QString>{""};}

    /*!
     * \brief Required method stub for checking that a file is loadable
     * \param[in] filePath Path to the file to be loaded
     * \param[in] plugins Map of plugins available
     * \return True
     */
    virtual bool canLoadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If*> plugins){ return true; }

    /*!
     * \brief Empty function that could be used to get user options
     * \param[in] filePath Path to the file to be loaded
     * \param[in] plugins Map of plugins available
     */
    virtual void getUserOptions(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If*> plugins){}

    /*!
     * \brief Creates a set of default objects and returns them instead of reading a file
     * \param[in] filePath Path to a file
     * \param[in] plugins Map of plugins availalbe
     * \return A vector of default objects
     */
    virtual QVector<WorldObject*> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins);

    /*!
     * \brief Builds the default differential drive robot
     * The exact robot varies depending on testing usage, but it usually
     * contains a circular body, a ring of touch sensors, and a lidar
     * \param[in] plugins Map of plugins availalbe
     * \return The default differential drive robot object
     */
    WorldObject* makeDiffDriveBot(QMap<QString, WorldObjectComponent_Plugin_If*> plugins);

    /*!
     * \brief Builds the default ackermann steered robot
     * The exact robot varies depending on testing usage, but it usually
     * contains a rectangular body, and a lidar
     * \param[in] plugins Map of plugins availalbe
     * \return The default ackermann steering robot object
     */
    WorldObject* makeAckermannBot(QMap<QString, WorldObjectComponent_Plugin_If*> plugins);
};
