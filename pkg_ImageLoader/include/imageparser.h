#ifndef IMAGEPARSE_H
#define IMAGEPARSE_H

#include <QVector>
#include <QPolygon>
#include <QImage>
#include <QRgb>

#ifdef WINDOWS
#include <windows.h>
#undef min
#undef max
#include <GL/GLU.h>
#else
#include <GL/glu.h>
#endif

#include <functional>

class ImageParser
{
public:
    struct Shape
    {
        QPolygonF outer;
        QVector<QPolygonF> inner;
    };

    static QVector<QVector<QPolygonF>> parseImage(QString fileName, const uint64_t &colorThreshold, const uint64_t &crossThreshold, uint64_t& width, uint64_t& height);
    static QVector<QVector<QPolygonF>> parseImage(const QImage& image, const uint64_t& colorThreshold, const uint64_t &crossThreshold);
    static QVector<QVector<QPolygonF>> parseImage(const QVector<QVector<QRgb> >& pixMap, const uint64_t& colorThreshold, const uint64_t &crossThreshold);

private:
    //Edge finding
    static QVector<QVector<QPolygonF> > _findShapes(QVector<QVector<bool> > &bwImage, const uint64_t &crossThreshold);
    static QPair<int64_t, int64_t> _moveShape(QVector<QVector<bool> >& bwImage, QVector<QVector<bool>>& copy, int64_t x, int64_t y);
    static Shape _findPoints(QVector<QVector<bool>>& shape);
    static void _bfsBlackWhite(const QVector<QVector<bool> > &bwImage, std::function<void(int, int)> handler, int64_t x, int64_t y, bool color);
    static QPolygonF _followBoundary(const QVector<QVector<bool>>& bwImage, int64_t x, int64_t y, bool outer= true);

    //Triangulating
    static QVector<QPolygonF> _triangulate(const Shape& s);
    static QVector<QPolygonF> _triangulateGLU(const Shape& s);

    //Polygon simplify
    static void _simplify(Shape &s, const uint64_t &crossThreshold);
    static QPolygonF _simplify(const QPolygonF& p, const uint64_t &crossThreshold);

    //Transform shape back to world space
    static void _transform(Shape& s, int width, int height, double scale);
    static void _transform(QPolygonF& p, int width, int height, double scale);

};

#endif // IMAGEPARSE_H
