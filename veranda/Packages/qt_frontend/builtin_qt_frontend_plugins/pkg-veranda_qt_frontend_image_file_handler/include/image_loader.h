//! \file
#pragma once

#include <QVector>
#include <QString>
#include <QSharedPointer>

#include <veranda_qt_frontend/object_loader_if.h>
#include <veranda_core/api/world_object.h>
#include <Box2D/Box2D.h>

#include "optiondialog.h"
#include "imageparser.h"

/*!
 * \brief File loader for converting image files to a set of triangles
 *
 * This loader can open any of the major image file formats (png, jpg, bmp) and
 * convert it to a set of shapes that can be loaded into the physics engine.
 *
 * If the image is not black and white, every pixel will be changed to black or white,
 * depending on its intensity. Once the image is all black and white, edges will
 * be found to form arbitrary polygons with holes inside of them. Those holes are
 * given to the polygon component from the shapes package.
 */
class ImageLoader : public WorldLoader_If
{
private:
    //! Record of the last options dialog presented to the user
    QSharedPointer<ImageOptions> lastOptions;

    /*!
     * \brief Reads a file and converts it to a set of Shape types
     * \param[in] filePath Path to the file to read
     * \param[in] colorThreshold Intensity threshold for black and white conversion. Pixels above this become 255, and below become 0
     * \return 0 or more Shape types
     */
    QVector<ImageParser::Shape> getShapesFromFile(QString filePath, uint64_t colorThreshold);

public:
    /*!
     * \brief Checks if a file can be loaded by this loader
     * \param[in] filePath Path to the file to check
     * \param[in] plugins Map of all plugins by their IID
     * \return true if the file can be loaded by this loader
     */
    virtual bool canLoadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If*> plugins);

    /*!
     * \brief Presents the user with a dialog to choose options for loading files
     * \param[in] filePath Path to the file to check
     * \param[in] plugins Map of all plugins by their IID
     */
    virtual void getUserOptions(QString filePath, QMap<QString, WorldObjectComponent_Factory_If*> plugins);

    /*!
     * \brief Returns the file types this loader can handle
     * \return "Black and White Image (*.png *.jpg *.jpeg *.bmp)"
     */
    virtual QVector<QString> fileExts() { return QVector<QString>{"Black and White Image (*.png *.jpg *.jpeg *.bmp)"};}

    /*!
     * \brief Loads an image and converts it to a series of WorldObjects
     * \param[in] filePath Path to the file to check
     * \param[in] plugins Map of all plugins by their IID
     * \return 0 or more WorldObject obstacles representing the dark space in the image
     */
    virtual QVector<WorldObject*> loadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If *> plugins);
};
