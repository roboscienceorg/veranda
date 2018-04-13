//! \file
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

//! Custom datatype for GLU Tesselator callbacks
struct GLTessData
{
    //! Current collection of vertices
    QVector<double*> vertices;

    //! Collection of all triangles thus far
    QVector<QPolygonF> triangles;

    //! Flag for if an error occurred
    bool error = false;
};

/*!
 * \brief Callback when tesselation begins
 * Pushes an empty triangle into the tesselation data so that GLTess_vert does access invalid memory
 * \param[in] type Type of object being tesselated; should always be GL_TRIANGLES
 * \param[in,out] data Custom data pointer - Points to GLTessData
 */
void GLTess_begin( GLenum type, void* data );

/*!
 * \brief Callback to add a vertex
 * Vertices are added to triangles.last() of the GLTessData passed. Therefore, the triangles
 * field of the struct should not be empty.
 * \param[in] vert The vertex added - Points to a double[3]
 * \param[in,out] data Custom data pointer - Points to GLTessData
 */
void GLTess_vert( void* vert, void* data );

/*!
 * \brief Collision resolution callback
 * This callback is required so GLTess can combine vertices of the original
 * polygon that are very close to each other; no extra processing
 * is done here for triangulation
 * \param[in] coords Coordinates of the new point
 * \param[in] verts The vertices that are resolved together
 * \param[in] weight Weights of the vertices being resolved
 * \param[out] out Pointer to be filled with the result point object
 * \param[in,out] data Custom data pointer - Points to GLTessData
 */
void GLTess_combine( GLdouble coords[3], void* verts[4],
                     GLfloat weight[4], void** out, void* data );

/*!
 * \brief Callback when triangulation encounters an error
 * \param[in] error The type of error that occurred
 * \param[in,out] data Custom data pointer - Points to GLTessData
 */
void GLTess_err( GLenum error, void* data );

/*!
 * \brief Dummy callback required to force triangulator to create GL_TRIANGLES only
 * \param[in] flag Unused
 * \param[in,out] data Custom data pointer - Points to GLTessData
 */
void GLTess_edge( GLboolean flag, void* data ){}

/*!
 * \brief
 * \param s
 * \return
 */
QVector<QPolygonF> triangulate(const Shape& s)
{
    //Storage for results and pointers
    GLTessData tData;

    //Make tesselator
    GLUtesselator* tess = gluNewTess();
    if(!tess) throw std::exception();

    //Set up callbacks

#ifdef WINDOWS
//! Typedef for GLU callback functions
#define CB (void (CALLBACK *)())
#else
//! Typedef for GLU callback functions
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

