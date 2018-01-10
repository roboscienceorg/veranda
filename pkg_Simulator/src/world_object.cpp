#include "world_object.h"

#include <QSet>

WorldObject::WorldObject(QVector<WorldObjectComponent_If *> components, QObject *parent) : QObject(parent), _components(components)
{
    QMap<QString, int> groupcounts;

    //Quick tally of if any components use the same group name
    for(WorldObjectComponent_If* c : _components)
    {
        groupcounts[c->getPropertyGroup()]++;
    }

    QSet<QString> multiples;
    for(auto iter = groupcounts.begin(); iter != groupcounts.end(); iter++)
        if(iter.value() > 1) multiples.insert(iter.key());

    //Initialize all aggregates
    for(WorldObjectComponent_If* c : _components)
    {
       /*************
        * Parenting
        * Auto-delete components when world object dies
        *************/
        c->setParent(this);

       /*************
        * Properties
        *************/
        //Get properties of child
        QMap<QString, PropertyView> props = c->getProperties();

        //If more than 1 child uses group, append a number
        //to the group
        QString group = c->getPropertyGroup();

        if(multiples.contains(group))
            group += QString::number(groupcounts[group]--);

        //Add all properties to property map
        for(auto iter = props.begin(); iter != props.end(); iter++)
        {
            _properties[group + "/" + iter.key()] = iter.value();
        }

       /*************
        * Models
        *************/
        _models += c->getModels();

       /*************
        * Channels
        *************/
        if(c->usesChannels()) _useChannels = true;
    }

    debugModel = new Model();
    debugModel2 = new Model();
    _models += debugModel;
    _models += debugModel2;
}

WorldObject* WorldObject::clone(QObject *newParent)
{
    QVector<WorldObjectComponent_If*> childClones;
    for(WorldObjectComponent_If* c : _components)
        childClones.push_back(c->clone());

    WorldObject* copy = new WorldObject(childClones, newParent);
    copy->_objName.set(_objName.get());

    return copy;
}

void WorldObject::clearBodies(b2World *world)
{
    QVector<WorldObjectComponent_If*> components = getComponents();

    if(anchorBody)
    {
        for(WorldObjectComponent_If* c : components)
            c->clearBodies(world);

        world->DestroyBody(anchorBody);
        anchorBody = nullptr;
    }
}

void WorldObject::generateBodies(b2World* world, object_id oId)
{
    clearBodies(world);

    QVector<WorldObjectComponent_If*> components = getComponents();

    b2BodyDef anchorDef;
    anchorDef.type = b2_dynamicBody;
    anchorDef.position.Set(0,0);
    anchorDef.allowSleep = false;
    anchorDef.fixedRotation = false;
    anchorBody = world->CreateBody(&anchorDef);

    anchorDef.position.Set(1, 0);
    tempBody = world->CreateBody(&anchorDef);

    b2CircleShape* circ = new b2CircleShape;
    circ->m_p = b2Vec2(0, 0);
    circ->m_radius = 1;

    b2EdgeShape* line = new b2EdgeShape;
    line->m_vertex1 = b2Vec2(0, 0);
    line->m_vertex2 = b2Vec2(0, 1);

    b2FixtureDef fix;
    fix.shape = circ;
    fix.density = 1;
    anchorBody->CreateFixture(&fix);

    fix.shape = line;
    fix.density = 1;
    tempBody->CreateFixture(&fix);

    b2WeldJointDef weld;
    weld.Initialize(anchorBody, tempBody, anchorBody->GetWorldCenter());
    weld.collideConnected = false;
    weld.frequencyHz = 0;
    world->CreateJoint(&weld);

    debugModel->addShapes(QVector<b2Shape*>{circ, line});
    debugModel2->addShapes(QVector<b2Shape*>{line});

    anchorBody->ApplyLinearImpulseToCenter(b2Vec2(0, 3), true);
    anchorBody->ApplyAngularImpulse(10, true);

    //for(int i = 0; i < components.size(); i++)
        //components[i]->generateBodies(world, oId, anchorBody);
}

void WorldObject::connectChannels()
{
    if(_useChannels)
    {
        for(WorldObjectComponent_If* c : _components)
            c->connectChannels();
    }
}

void WorldObject::disconnectChannels()
{
    if(_useChannels)
    {
        for(WorldObjectComponent_If* c : _components)
            c->disconnectChannels();
    }
}

void WorldObject::worldTicked(const b2World* w, const double t)
{
    qDebug() << "Main Object";
    qDebug() << anchorBody->GetPosition().x << ", " << anchorBody->GetPosition().y;
    qDebug() << anchorBody->GetLinearVelocity().x << ", " << anchorBody->GetLinearVelocity().y << " : " << anchorBody->GetAngularVelocity();

    debugModel->setTransform(anchorBody->GetPosition().x, anchorBody->GetPosition().y, anchorBody->GetAngle()*(180/(22.0/7)));
    debugModel2->setTransform(tempBody->GetPosition().x, tempBody->GetPosition().y, tempBody->GetAngle()*(180/(22.0/7)));

    //for(WorldObjectComponent_If* c : _components)
        //c->worldTicked(w, t);
}
