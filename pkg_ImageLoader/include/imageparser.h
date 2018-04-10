//! \file
#pragma once

#include <QVector>
#include <QPolygon>
#include <QImage>
#include <QRgb>

#include <functional>

/*!
 * \brief A grouping of static functions for processing images
 * The ImageParser class contains a set of functions used by the
 * image loader to read images and convert them to a set of shapes. Most
 * of the internal functions are private, and three entry functions
 * that can be used exist
 */
class ImageParser
{
public:
    //! Representation of a shape as an outer loop and some number of inner loops (holes)
    struct Shape
    {
        QPolygonF outer; //!< The outer loop of the shape
        QVector<QPolygonF> inner; //!< Some number of loops representing holes in the shape
    };

    /*!
     * \brief Converts a file to a set of shapes
     * \param[in] fileName Path to the file to read
     * \param[in] colorThreshold Threshold for intensity to be either black or white
     * \param[out] width Width of the image loaded
     * \param[out] height Height of the image loaded
     * \return A QVector containing all the distinct shapes found in the image
     */
    static QVector<Shape> parseImage(QString fileName, const uint64_t &colorThreshold, uint64_t& width, uint64_t& height);

    /*!
     * \brief Converts an image to a set of shapes
     * \param[in] image A QImage loaded with data
     * \param[in] colorThreshold Threshold for intensity to be either black or white
     * \return A QVector containing all the distinct shapes found in the image
     */
    static QVector<Shape> parseImage(const QImage& image, const uint64_t& colorThreshold);

    /*!
     * \brief Converts a pixelmap to a set of shapes
     * \param[in] pixMap A 2D Vector of RGB pixels
     * \param[in] colorThreshold Threshold for intensity to be either black or white
     * \return A QVector containing all the distinct shapes found in the image
     */
    static QVector<Shape> parseImage(const QVector<QVector<QRgb> >& pixMap, const uint64_t& colorThreshold);

private:
    /*!
     * \brief Parses a black and white image to a set of distinct shapes with holes
     * \param[in] bwImage The image to parse as a 2D vector of booleans (black = false, white = true)
     * \return A QVector containing all the distinct shapes found in the image
     */
    static QVector<Shape> _findShapes(QVector<QVector<bool> > &bwImage);

    /*!
     * \brief Copies a connected white section of an image into its own small image and erases it from the origin image
     * \param[in/out] bwImage The image to copy a connected section from
     * \param[out] copy A 2D array containing only pixels of the connected section
     * \param[in] x x coordinate of a point in the shape
     * \param[in] y y coordinate of a point in the shape
     * \return An x, y pair the upper left corner of the bounding box of the shape in the original image
     */
    static QPair<int64_t, int64_t> _moveShape(QVector<QVector<bool> >& bwImage, QVector<QVector<bool>>& copy, int64_t x, int64_t y);

    /*!
     * \brief Converts a connected section of white pixels into sequences of points representing its loops
     * \param[in] shape The set of points to convert
     * \return A Shape type; the outer loop of the connected section and any of its inner loops
     */
    static Shape _findPoints(QVector<QVector<bool>>& shape);

    /*!
     * \brief Performs a breadth-first search on a group of connected black or white pixels, triggering a callback for each
     * \param[in] bwImage Image to bfs on
     * \param[in] handler Callback function to trigger on each connected pixel
     * \param[in] x Starting x coordinate
     * \param[in] y Starting y coordinate
     * \param[in] color Color search for
     */
    static void _bfsBlackWhite(const QVector<QVector<bool> > &bwImage, std::function<void(int, int)> handler, int64_t x, int64_t y, bool color);

    /*!
     * \brief Walks along the edge of a shape and records the pixel locations
     * \param[in] bwImage Image containing the shape to follow
     * \param[in] x Starting x coordinate
     * \param[in] y Starting y coordinate
     * \param[in] outer Flag to indicate which direction to go (clockwise for outer loop, counterclockwise for inner)
     * \return The set of points that are on the edge
     */
    static QPolygonF _followBoundary(const QVector<QVector<bool>>& bwImage, int64_t x, int64_t y, bool outer= true);

    /*!
     * \brief Centers a shape and scales it
     * \param[in/out] s Shape to transform
     * \param[in] width Width to center based on
     * \param[in] height Height to center based on
     * \param[in] scale Scaling factor to apply
     */
    static void _transform(Shape& s, int width, int height, double scale);

    /*!
     * \brief Centers a point loop and scales it
     * \param[in/out] s Points to transform
     * \param[in] width Width to center based on
     * \param[in] height Height to center based on
     * \param[in] scale Scaling factor to apply
     */
    static void _transform(QPolygonF& p, int width, int height, double scale);
};
