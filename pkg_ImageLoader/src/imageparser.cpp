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

QVector<QVector<QPolygonF>> ImageParser::parseImage(QString fileName, const uint64_t& threshold, uint64_t &width, uint64_t &height)
{
    qDebug() << "Opening file...";
    QImage loaded(fileName);

    if(loaded.isNull())
        throw std::exception();

    width = loaded.width();
    height = loaded.height();

    return parseImage(loaded, threshold);
}

QVector<QVector<QPolygonF>> ImageParser::parseImage(const QImage& image, const uint64_t& threshold)
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

    return parseImage(pixMap, threshold);
}

QVector<QVector<QPolygonF> > ImageParser::parseImage(const QVector<QVector<QRgb>>& pixMap, const uint64_t& threshold)
{
    qDebug() << "Making black and white...";
    if(!pixMap.size() || !pixMap[0].size())
        throw std::exception();

    QVector<QVector<bool>> blackWhite(pixMap.size(), QVector<bool>(pixMap[0].size()));
    for(int i=0; i<pixMap.size(); i++)
    {
        for(int j=0; j<pixMap[i].size(); j++)
        {
            blackWhite[i][j] = qGray(pixMap[i][j]) > threshold;
        }
    }

    return _findShapes(blackWhite);
}

QVector<QVector<QPolygonF>> ImageParser::_findShapes(QVector<QVector<bool> >& bwImage)
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
        qDebug() << double(i*bwImage[0].size())/(bwImage.size()*bwImage[0].size()) * 100 << "%";
    }

    QVector<QVector<QPolygonF>> out;

    qDebug() << "Simplify and triangulate";
    for(Shape& s : shapes)
    {
        //qDebug() << "Simplify";
        _simplify(s);

        _transform(s, bwImage[0].size(), bwImage.size(), 0.5);

        //qDebug() << "Triangulate";
        out += _triangulate(s);
    }

    return out;
}

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

void ImageParser::_simplify(Shape &s)
{
    s.outer = _simplify(s.outer);
    for(QPolygonF& p : s.inner)
        p = _simplify(p);
}

QPolygonF ImageParser::_simplify(const QPolygonF &p)
{
    if(p.size() <= 3) return p;

    const double EPSILON = 0.001;

    //Cross product of vectors AB, AC using points A, B, C
    const std::function<double(const QPointF&, const QPointF&, const QPointF&)> cross =
    [](const QPointF& A, const QPointF& B, const QPointF& C)
    {
        QPointF AB(B.x() - A.x(), B.y() - A.y());
        QPointF AC(C.x() - A.x(), C.y() - A.y());
        return AB.x() * AC.y() - AB.y() * AC.x();
    };

    QPolygonF out;

    //Assume point 0 is a corner
    QPointF pointA = p[0];
    uint64_t indexA = 0;
    uint64_t indexB = 1;
    uint64_t indexC = 2;

    bool lastCorner = false, firstPoint = true;
    uint64_t firstIndex = 0;

    //Walk two consecutive points around the polygon
    //Checking the cross product between them and the last corner
    //If it's non-0, then B is a new corner
    do
    {
        //qDebug() << indexA << indexB << indexC;
        //qDebug() << indexB/(double)p.size() * 100;
        double cProd = abs(cross(pointA, p[indexB], p[indexC]));
        if(cProd > EPSILON)
        {
            //qDebug() << "Corner";

            indexA = indexB;
            pointA = p[indexA];

            //Don't double-push first point
            if(indexA != firstIndex)
                out.push_back(pointA);

            if(lastCorner) break;

            //If this is the first corner found
            //set flags so we don't exit,
            //and mark the corner
            if(firstPoint)
            {
                firstPoint = false;
                firstIndex = indexA;
            }
        }
        indexB = (indexB+1) % p.size();
        indexC = (indexC+1) % p.size();

        if(indexB == 0) lastCorner = true;

        //If the entire thing is a line, or we can't otherwise simplify it
        //break out of infinite loop
        if(indexC == indexA) return p;

    //Continue around until the first corner is found
    //a second time
    }while(true);

    return out;
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

QVector<QPolygonF> ImageParser::_triangulate(const Shape& s)
{
    /* Ignore triangulation; use only for debugging pre-triangle steps
    QVector<QPolygonF> out_ = s.inner;
    out_ += s.outer;
    return out_;
    */

    return _triangulateGLU(s);
}

struct GLTessData
{
    QVector<double*> vertices;
    QVector<QPolygonF> triangles;
    bool error = false;
};

void GLTess_begin( GLenum type, void* data );
void GLTess_vert( void* vert, void* data );
void GLTess_combine( GLdouble coords[3], void* verts[4],
                     GLfloat weight[4], void** out, void* data );
void GLTess_err( GLenum error, void* data );
void GLTess_edge( GLboolean flag, void* data ){}

QVector<QPolygonF> ImageParser::_triangulateGLU(const Shape& s)
{
    //Storage for results and pointers
    GLTessData tData;

    //Make tesselator
    GLUtesselator* tess = gluNewTess();
    if(!tess) throw std::exception();

    //Set up callbacks

#ifdef WINDOWS
#define CB (void (CALLBACK *)())
#else
#define CB (_GLUfuncptr)
#endif
    gluTessCallback(tess, GLU_TESS_BEGIN_DATA, CB GLTess_begin);
    gluTessCallback(tess, GLU_TESS_COMBINE_DATA, CB GLTess_combine);
    gluTessCallback(tess, GLU_TESS_VERTEX_DATA,  CB GLTess_vert);
    gluTessCallback(tess, GLU_TESS_ERROR_DATA,  CB GLTess_err);

    //Specify empty edge function to force only triangles
    gluTessCallback(tess, GLU_TESS_EDGE_FLAG_DATA, CB GLTess_edge);
#undef CB

    //Add shape as polygon with multiple contours
    //Assume inner shapes are already opposite winding of outer
    gluTessBeginPolygon(tess, &tData);
        gluTessBeginContour(tess);
        for(const QPointF& p : s.outer)
        {
            double* pnt = new double[3];
            pnt[0] = p.x();
            pnt[1] = p.y();
            pnt[2] = 0;

            tData.vertices.push_back(pnt);
            gluTessVertex(tess, pnt, pnt);
        }
        gluTessEndContour(tess);

    for(const QPolygonF& poly : s.inner)
    {
        gluTessBeginContour(tess);
        for(const QPointF& p : poly)
        {
            double* pnt = new double[3];
            pnt[0] = p.x();
            pnt[1] = p.y();
            pnt[2] = 0;

            tData.vertices.push_back(pnt);
            gluTessVertex(tess, pnt, pnt);
        }
        gluTessEndContour(tess);
    }
    gluTessEndPolygon(tess);

    //Clean up
    gluDeleteTess(tess);

    if(tData.error)
        tData.triangles.clear();

    for(double* d : tData.vertices)
        delete[] d;

    return tData.triangles;
}

void GLTess_begin( GLenum type, void* data )
{
    GLTessData* tData = static_cast<GLTessData*>(data);

    tData->triangles.push_back(QPolygonF());
}

void GLTess_vert( void* vert, void* data )
{
    GLTessData* tData = static_cast<GLTessData*>(data);
    double* vData = (double*)vert;

    if(tData->triangles.last().size() == 3)
        tData->triangles.push_back(QPolygonF());

    tData->triangles.last().push_back(QPointF(vData[0], vData[1]));
}

void GLTess_combine( GLdouble coords[3], void* verts[4],
                     GLfloat weight[4], void** out, void* data )
{
    GLTessData* tData = static_cast<GLTessData*>(data);

    double* out_ = new double[3];
    out_[0] = coords[0];
    out_[1] = coords[1];
    out_[2] = 0;

    tData->vertices.push_back(out_);
    *out = out_;
}

void GLTess_err( GLenum error, void* data )
{
    GLTessData* tData = static_cast<GLTessData*>(data);
    tData->error = error;
}
