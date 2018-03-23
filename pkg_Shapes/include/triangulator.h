#ifndef TRIANGULATOR_H
#define TRIANGULATOR_H

#include <QPolygonF>

#ifdef WINDOWS
#include <windows.h>
#undef min
#undef max
#include <GL/GLU.h>
#else
#include <GL/glu.h>
#endif

struct Shape
{
    QPolygonF outer;
    QVector<QPolygonF> inner;
};

//Triangulating
QVector<QPolygonF> triangulate(const Shape& s);
QVector<QPolygonF> triangulateGLU(const Shape& s);

//Polygon simplify
void simplify(Shape &s, const uint64_t &crossThreshold);
QPolygonF simplify(const QPolygonF& p, const uint64_t &crossThreshold);
#endif // TRIANGULATOR_H
