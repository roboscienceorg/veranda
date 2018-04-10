#include "imageparser.h"

#include <stdexcept>

#include <QVector>
#include <QImage>
#include <QRgb>
#include <QDebug>
#include <QQueue>

#include <algorithm>
#include <vector>
#include <cmath>

typedef ImageParser::Shape Shape;
using std::vector;

/*!
 * \throws std::exception - A null image is loaded
 */
QVector<ImageParser::Shape> ImageParser::parseImage(QString fileName, const uint64_t& colorThreshold, uint64_t &width, uint64_t &height)
{
    qDebug() << "Opening file...";
    QImage loaded(fileName);

    if(loaded.isNull())
        throw std::exception();

    width = loaded.width();
    height = loaded.height();

    return parseImage(loaded, colorThreshold);
}

QVector<ImageParser::Shape> ImageParser::parseImage(const QImage& image, const uint64_t& colorThreshold)
{
    qDebug() << "Reading image colors...";
    QVector<QVector<QRgb>> pixMap(image.height(), QVector<QRgb>(image.width()));

    for(int i=0; i<image.height(); i++)
    {
        for(int j=0; j<image.width(); j++)
        {
            pixMap[i][j] = image.pixel(j, i);
        }
    }

    return parseImage(pixMap, colorThreshold);
}

/*!
 * This function assumes that the input pixmap is rectangular (not jagged)
 *
 * \throws std::exception - The pixel map has 0 size
 */
QVector<Shape> ImageParser::parseImage(const QVector<QVector<QRgb>>& pixMap, const uint64_t& colorThreshold)
{
    qDebug() << "Making black and white...";
    if(!pixMap.size() || !pixMap[0].size())
        throw std::exception();

    QVector<QVector<bool>> blackWhite(pixMap.size(), QVector<bool>(pixMap[0].size()));
    for(int i=0; i<pixMap.size(); i++)
    {
        for(int j=0; j<pixMap[i].size(); j++)
        {
            blackWhite[i][j] = qGray(pixMap[i][j]) > colorThreshold;
        }
    }

    return _findShapes(blackWhite);
}

QVector<Shape> ImageParser::_findShapes(QVector<QVector<bool> >& bwImage)
{
    qDebug() << "Searching for shapes...";
    QVector<QVector<bool>> singleShape;
    QVector<Shape> shapes;

    const std::function<void(QPolygonF&, QPair<int64_t, int64_t>)> shift =
    [](QPolygonF& poly, QPair<int64_t, int64_t> offset)
    {
        for(QPointF& p : poly)
        {
            p.setX(p.x() + offset.first);
            p.setY(p.y() + offset.second);
        }
    };

    int lastPct = 0;
    for(int i=0; i<bwImage.size(); i++)
    {
        for(int j=0; j<bwImage[i].size(); j++)
        {
            if(!bwImage[i][j])
            {
                QPair<int64_t, int64_t> offset = _moveShape(bwImage, singleShape, j, i);
                //qDebug() << "Moved shape offset at: " << offset.first << offset.second;
                Shape newShape = _findPoints(singleShape);

                shift(newShape.outer, offset);
                for(QPolygonF& p : newShape.inner)
                    shift(p, offset);
                shapes += newShape;
            }
        }
        if(lastPct + 4 < (int)(double(i*bwImage[0].size())/(bwImage.size()*bwImage[0].size()) * 100))
            qDebug() << (lastPct = (int)(double(i*bwImage[0].size())/(bwImage.size()*bwImage[0].size()) * 100)) << "%";
    }

    for(Shape& s : shapes)
        _transform(s, bwImage[0].size(), bwImage.size(), 0.5);


    return shapes;
}

/*!
 * The callback function follows the prototype <void (int, int)>. It is called for each point
 * touched by the bfs with the parameters x, y
 */
void ImageParser::_bfsBlackWhite(const QVector<QVector<bool> >& bwImage, std::function<void (int, int)> handler, int64_t x, int64_t y, bool color)
{
    QVector<QVector<bool>>visited (bwImage.size(), QVector<bool>(bwImage[0].size(), false));
    QQueue<QPair<int64_t, int64_t>> queue;
    queue += {x, y};

    while(queue.size())
    {
        QPair<int64_t, int64_t> pnt = queue.dequeue();

        //Check that point is in bounds
        if(pnt.second >= 0 && pnt.second < bwImage.size() &&
           pnt.first >= 0 && pnt.first < bwImage[pnt.second].size() &&

        //Check that point is the target color, and unvisited
           bwImage[pnt.second][pnt.first] == color && !visited[pnt.second][pnt.first])
        {
            //Mark visited
            visited[pnt.second][pnt.first] = true;

            //Trigger handler function
            handler(pnt.first, pnt.second);

            //Move to neighbors
            queue += {pnt.first+1, pnt.second};
            queue += {pnt.first-1, pnt.second};
            queue += {pnt.first, pnt.second+1};
            queue += {pnt.first, pnt.second-1};
        }
    }
}

QPair<int64_t, int64_t> ImageParser::_moveShape(QVector<QVector<bool> > &bwImage, QVector<QVector<bool> > &copy, int64_t x, int64_t y)
{
    int64_t minx = bwImage[0].size(), miny = bwImage.size();
    int64_t maxx = 0, maxy = 0;

    //Run bfs to find size of the black region
    _bfsBlackWhite(bwImage,
    [&](int64_t x_, int64_t y_)
    {
        minx = std::min(minx, x_);
        miny = std::min(miny, y_);
        maxx = std::max(maxx, x_);
        maxy = std::max(maxy, y_);
    },
    x, y, false);

    //Create block to copy to
    //A white block with 1px border on top and bottom
    int64_t height = maxy - miny + 1;
    int64_t width = maxx - minx + 1;

    copy = QVector<QVector<bool>>(height+2, QVector<bool>(width+2, true));

    //Run bfs again to do copy
    _bfsBlackWhite(bwImage,
    [&](int64_t x_, int64_t y_)
    {
        //Remove shape pixel from master image
        bwImage[y_][x_] = true;

        //Add shape pixel to copy image
        copy[y_-miny+1][x_-minx+1] = false;
    },
    x, y, false);

    return {minx-1, miny-1};
}

Shape ImageParser::_findPoints(QVector<QVector<bool> > &shape)
{
    bool firstEdge = true;
    Shape out;

    for(int i=0; i<shape.size(); i++)
    {
        for(int j=0; j<shape[i].size(); j++)
        {
            //Check if point is white, and at least one adjacent is black
            if(shape[i][j] &&
             ((i != 0 && !shape[i-1][j]) || (i != shape.size()-1 && !shape[i+1][j]) ||
              (j != 0 && !shape[i][j-1]) || (j != shape[i].size()-1 && !shape[i][j+1])))
            {
                QPolygonF bound = _followBoundary(shape, j, i, firstEdge);

                //First edge we find is the outer loop of the shape
                if(firstEdge)
                {
                    out.outer = bound;
                }
                else
                {
                    //Reverse winding for inner polygons
                    //Don't need to; should already be reversed due to algorithm
                    //std::reverse(bound.begin(), bound.end());
                    out.inner += bound;
                }

                //All other loops are inner loops
                firstEdge = false;

                //Erase the whitespace we're currently looking at
                _bfsBlackWhite(shape, [&](int64_t x_, int64_t y_){shape[y_][x_] = false;}, j, i, true);
            }
        }
    }

    return out;
}

/*!
 * This is the key part of the image parsing. It works similarly to a breadth first or depth first search, but
 * the queue is always size 1; only black pixels are considered. At each point, the 8 points surrounding are checked.
 * Of those points, only the ones with 3 or more black neighbors may be the next point. The first point found which
 * satisfies these constraints is stepped to as the next point. When searching for a neighbor of that point,
 * we consider the 8 points around it starting with the one in the same direction that was used to get to it.
 */
QPolygonF ImageParser::_followBoundary(const QVector<QVector<bool>>& bwImage, int64_t x, int64_t y, bool outer)
{
    const std::function<bool(const QVector<QVector<bool>>&, int64_t, int64_t)> isBlack =
    [](const QVector<QVector<bool>>& img_, int64_t j, int64_t i)
    {
        bool out = false;
        if(i >= 0 && i < img_.size() && j >= 0 && j < img_[i].size())
        {
            //qDebug() << "Point" << i <<"," << j <<":" << img_[i][j];
            out = img_[i][j] == false;
        }

        //qDebug() << "Point" << i <<"," << j << "is black:" << out;
        return out;
    };

    //Checks if a point adjacent is black
    //Assumes that non-existant points are white
    const std::function<QVector<QPair<int64_t, int64_t>>(const QVector<QVector<bool>>&, int64_t, int64_t)> adjacentBlacks =
    [&isBlack](const QVector<QVector<bool>>& img_, int64_t j, int64_t i)
    {
        QVector<QPair<int64_t, int64_t>> out;
        for(int i_=-1; i_<=1; i_++)
        {
            for(int j_=-1; j_<=1; j_++)
            {
                if(i_ != 0 || j_!= 0)
                {
                    if(isBlack(img_, j+j_, i+i_)) out += {j+j_, i+i_};
                }
            }
        }
        return out;
    };

    //Cross product of vectors AB, AC using points A, B, C
    const std::function<int64_t(const QPair<int64_t, int64_t>&, const QPair<int64_t, int64_t>&, const QPair<int64_t, int64_t>&)> cross =
    [](const QPair<int64_t, int64_t>& A, const QPair<int64_t, int64_t>& B, const QPair<int64_t, int64_t>& C)
    {
        QPair<int64_t, int64_t> AB{B.first - A.first, B.second - A.second};
        QPair<int64_t, int64_t> AC{C.first - A.first, C.second - A.second};
        return AB.first * AC.second - AB.second * AC.first;
    };

    //Order of searching points so we stay going clockwise or counter clockwise
    QVector<QPair<int64_t, int64_t>> search{{1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}};
    int64_t searchNext = 0;

    QPolygonF points;

    QPair<int64_t, int64_t> currPoint{x, y}, lastPoint = currPoint;
    do
    {
        points << QPoint(currPoint.first, currPoint.second);

        //qDebug() << "Add Point: " << currPoint.first << "," << currPoint.second;

        bool good = false;
        for(int i=0; i<search.size() && !good; i++)
        {
            if(searchNext < 0) searchNext = search.size()-1;
            if(searchNext >= search.size()) searchNext = 0;

            QPair<int64_t, int64_t> next{currPoint.first + search[searchNext].first, currPoint.second + search[searchNext].second};

            //qDebug() << "Check Point:" << searchNext << next.first << "," << next.second;

            //Check that we aren't stepping where we came from
            //if(next != lastPoint && next != currPoint)
            //{
                //qDebug() << "Not last or current point";

                //Check we're stepping to a white space
                if(next.second < 0 || next.second >= bwImage.size() || next.first < 0 || next.first >= bwImage[next.second].size() ||
                   bwImage[next.second][next.first])
                {
                    //qDebug() << "Is white";

                    //Get neighbors to potential point that are black
                    QVector<QPair<int64_t, int64_t>> blackNeighbors = adjacentBlacks(bwImage, next.first, next.second);

                    //Avoid going into tight spaces with no way out
                    //if(blackNeighbors.size() < 3)
                    //{
                        //Check the cross product from curr to next and neighbor of next
                        //If positive, it stays on the left side of the shape and we want
                        //to move there
                        for(QPair<int64_t, int64_t>& n : blackNeighbors)
                        {
                            int64_t cprod = cross(currPoint, next, n);
                            //qDebug() << "Cross: " << cprod;
                            if(cprod > 0)
                            {
                                //qDebug() << "Good!";
                                good = true;
                                lastPoint = currPoint;
                                currPoint = next;
                                searchNext = (searchNext + 4) % search.size();
                                //searchNext += (outer ? -2 : 2);
                                break;
                            }
                        }
                    //}
                }
            //}
            //if(!good)
                searchNext += (outer ? 1 : -1);
        }
    }while(currPoint.first != x || currPoint.second != y);

    //qDebug() << "End Loop";
    return points;
}

void ImageParser::_transform(Shape& s, int width, int height, double scale)
{
    _transform(s.outer, width, height, scale);
    for(QPolygonF& p : s.inner)
        _transform(p, width, height, scale);
}

void ImageParser::_transform(QPolygonF& p, int width, int height, double scale)
{
    QPointF shift(width/2, height/2);
    for(QPointF& pt : p)
    {
        pt -= shift;
        pt.setY(-pt.y());
        pt *= scale;
    }
}
