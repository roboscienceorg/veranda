#include "polygonscomponent.h"

PolygonsComponent::PolygonsComponent(QVector<b2PolygonShape *> polys, QObject* parent) : WorldObjectComponent_If(parent)
{
    _shapePtrs.resize(polys.size());

    for(int i=0; i<polys.size(); i++)
        _shapePtrs[i] = copy(polys[i]);

    _numShapes.set(_shapePtrs.size());

    _polyModel = new Model(QVector<Model*>{}, _shapePtrs, this);
}

PolygonsComponent::~PolygonsComponent()
{
    _polyModel->removeShapes(_polyModel->shapes());
    qDeleteAll(_shapePtrs);
    _shapePtrs.clear();
}

WorldObjectComponent_If* PolygonsComponent::clone(QObject* newParent)
{
    QVector<b2PolygonShape*> polyShapes;
    for(b2Shape* s : _shapePtrs)
        polyShapes += dynamic_cast<b2PolygonShape*>(s);

    PolygonsComponent* out = new PolygonsComponent(polyShapes, newParent);

    for(QString s : _properties.keys())
    {
        out->_properties[s].set(_properties[s].get(), true);
    }

    return out;
}

void PolygonsComponent::generateBodies(b2World* world, object_id oId, b2Body* anchor)
{
    clearBodies(world);

    b2BodyDef bDef;
    bDef.type = b2_staticBody;
    _polyBody = world->CreateBody(&bDef);

    moveBodyToLocalSpaceOfOtherBody(_polyBody, anchor, 0, 0, 0);

    for(b2Shape* p : _shapePtrs)
    {
        _polyFixtures += _polyBody->CreateFixture(p, 1);
    }
    massChanged(this, _polyBody->GetMass());
}

void PolygonsComponent::clearBodies(b2World* world)
{
    if(_polyBody)
    {
        for(b2Fixture* f : _polyFixtures)
            _polyBody->DestroyFixture(f);
        world->DestroyBody(_polyBody);

        _polyFixtures.clear();
    }
    massChanged(this, 0);
}

b2PolygonShape* PolygonsComponent::copy(b2PolygonShape* orig)
{
    b2PolygonShape* newPoly = new b2PolygonShape;
    *newPoly = *orig;

    return newPoly;
}
