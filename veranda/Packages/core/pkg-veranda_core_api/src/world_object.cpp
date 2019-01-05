#include "veranda_core/world_object.h"

#include <QSet>

WorldObject::WorldObject(QVector<WorldObjectComponent *> components, QString name, QString type, QObject *parent) : WorldObjectComponent(name, type, parent), _components(components)
{
    //Initialize all aggregates
    for(WorldObjectComponent* c : _components)
    {
       /*************
        * Parenting
        * Auto-delete components when world object dies
        *************/
        c->setParent(this);

        registerChild(c);
    }

    _debugModel = new Model();
    //registerModel(_debugModel);
}

WorldObject* WorldObject::_clone(QObject *newParent)
{
    QVector<WorldObjectComponent*> childClones;
    for(WorldObjectComponent* c : _components)
        childClones.push_back(c->clone());

    WorldObject* copy = new WorldObject(childClones, getName(), getType(), newParent);
    return copy;
}

void WorldObject::_clearBodies()
{
    if(!_world) return;

    unregisterBody(_anchorBody);
    _world->DestroyBody(_anchorBody);
    _anchorBody = nullptr;

    _world = nullptr;
}

void WorldObject::_generateBodies(b2World* world, object_id oId, b2Body* anchorBody)
{
    clearBodies();
    _world = world;

    //qDebug() << "Creating object at " << _locX.get().toDouble() << ", " << _locY.get().toDouble() << " : " << _locTheta.get().toDouble();

    QVector<WorldObjectComponent*> components = getComponents();

    b2BodyDef anchorDef;
    anchorDef.type = b2_dynamicBody;
    _anchorBody = world->CreateBody(&anchorDef);
    registerBody(_anchorBody, {_debugModel}, true);

    b2CircleShape* circ = new b2CircleShape;
    circ->m_p = b2Vec2(0, 0);
    circ->m_radius = 1;

    b2FixtureDef fix;
    fix.shape = circ;
    fix.density = 1;
    fix.isSensor = true;
    _anchorBody->CreateFixture(&fix);

    _debugModel->addShapes(QVector<b2Shape*>{circ});
}
