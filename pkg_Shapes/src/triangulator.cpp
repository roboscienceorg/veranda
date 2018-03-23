#include "triangulator.h"

#include <functional>
#include <cmath>

#include <QDebug>

void simplify(Shape &s, const uint64_t& crossThreshold)
{
    s.outer = simplify(s.outer, crossThreshold);
    for(QPolygonF& p : s.inner)
        p = simplify(p, crossThreshold);
}

QPolygonF simplify(const QPolygonF &p, const uint64_t& crossThreshold)
{
    if(p.size() <= 3) return p;

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
        double cProd = std::abs(cross(pointA, p[indexB], p[indexC]));
        if(cProd > crossThreshold)
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

QVector<QPolygonF> triangulate(const Shape& s)
{
    /* Ignore triangulation; use only for debugging pre-triangle steps
    QVector<QPolygonF> out_ = s.inner;
    out_ += s.outer;
    return out_;
    */

    return triangulateGLU(s);
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

QVector<QPolygonF> triangulateGLU(const Shape& s)
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

