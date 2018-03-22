#ifndef IMAGEPARSE_H
#define IMAGEPARSE_H

#include <QVector>
#include <QPolygon>
#include <QImage>
#include <QRgb>

#include <functional>

class ImageParser
{
public:
    struct Shape
    {
        QPolygonF outer;
        QVector<QPolygonF> inner;
    };

    static QVector<Shape> parseImage(QString fileName, const uint64_t &colorThreshold, uint64_t& width, uint64_t& height);
    static QVector<Shape> parseImage(const QImage& image, const uint64_t& colorThreshold);
    static QVector<Shape> parseImage(const QVector<QVector<QRgb> >& pixMap, const uint64_t& colorThreshold);

private:
    //Edge finding
    static QVector<Shape> _findShapes(QVector<QVector<bool> > &bwImage);
    static QPair<int64_t, int64_t> _moveShape(QVector<QVector<bool> >& bwImage, QVector<QVector<bool>>& copy, int64_t x, int64_t y);
    static Shape _findPoints(QVector<QVector<bool>>& shape);
    static void _bfsBlackWhite(const QVector<QVector<bool> > &bwImage, std::function<void(int, int)> handler, int64_t x, int64_t y, bool color);
    static QPolygonF _followBoundary(const QVector<QVector<bool>>& bwImage, int64_t x, int64_t y, bool outer= true);

    //Transform shape back to world space
    static void _transform(Shape& s, int width, int height, double scale);
    static void _transform(QPolygonF& p, int width, int height, double scale);
};

#endif // IMAGEPARSE_H
