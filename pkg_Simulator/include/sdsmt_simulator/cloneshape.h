#ifndef CLONESHAPE_H
#define CLONESHAPE_H

#include <Box2D/Box2D.h>

inline b2Shape* cloneShape(b2Shape* s)
{
    switch(s->GetType())
    {
    case b2Shape::e_circle:
    {
        b2CircleShape* in = static_cast<b2CircleShape*>(s);
        b2CircleShape* out = new b2CircleShape;
        out->m_radius = in->m_radius;
        out->m_p = in->m_p;
        return out;
    }
    case b2Shape::e_edge:
    {
        b2EdgeShape* in = static_cast<b2EdgeShape*>(s);
        b2EdgeShape* out = new b2EdgeShape;
        out->m_hasVertex0 = in->m_hasVertex0;
        out->m_hasVertex3 = in->m_hasVertex3;
        out->m_radius = in->m_radius;
        out->m_vertex0 = in->m_vertex0;
        out->m_vertex1 = in->m_vertex1;
        out->m_vertex2 = in->m_vertex2;
        out->m_vertex3 = in->m_vertex3;
        return out;
    }
    case b2Shape::e_polygon:
    {
        b2PolygonShape* in = static_cast<b2PolygonShape*>(s);
        b2PolygonShape* out = new b2PolygonShape;
        out->Set(in->m_vertices, in->m_count);
        out->m_centroid = in->m_centroid;
        out->m_radius = in->m_radius;
        return out;
    }
    default:
        return nullptr;
    }
}

#endif // CLONESHAPE_H
