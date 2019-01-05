//! \file
#pragma once

#include <QPolygonF>

//OpenGL is part of the Windows SDK, so when building
//on windows, we include that; but then undefine the min/max macros
//that it creates
#ifdef WINDOWS
#include <windows.h>
#undef min
#undef max
#include <GL/GLU.h>
#else
#include <GL/glu.h>
#endif

//! Container for an outer loop of a shape and some number of inner holes
struct Shape
{
    //! Outer loop of the shape
    QPolygonF outer;

    //! 0 or more inner loops that are holes in the outer shape
    QVector<QPolygonF> inner;
};

/*!
 * \brief Converts a shape with holes in it into a set of triangles
 * \param[in] s Shape to triangulate
 * \return A set of QPolygonF, each with 3 points. The points of each shape are not guaranteed unique.
 */
QVector<QPolygonF> triangulate(const Shape& s);

/*!
 * \brief Simplifies a shape by removing points between line segments that are almost colinear
 * \param[in] s Shape to simplify
 * \param[in] crossThreshold Threshold that cross products must be over to be considered non-colinear
 */
void simplify(Shape &s, const double &crossThreshold);

/*!
 * \brief Simplifies a line loop by removing points between line segments that are almost colinear
 * \param[in] p Polygon to simplify
 * \param[in] crossThreshold Threshold that cross products must be over to be considered non-colinear
 * \return The simplified polygon
 */
QPolygonF simplify(const QPolygonF& p, const double &crossThreshold);
