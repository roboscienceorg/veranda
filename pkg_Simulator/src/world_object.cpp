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

void WorldObject::generateBodies(b2World* world, object_id oId)
{
    QVector<WorldObjectComponent_If*> components = getComponents();

    b2BodyDef anchorDef;
    anchorDef.type = b2_dynamicBody;
    anchorDef.position.Set(0,0);
    b2Body* anchor = world->CreateBody(&anchorDef);

    for(int i = 0; i < components.size(); i++)
        components[i]->generateBodies(world, oId, anchor);
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
    for(WorldObjectComponent_If* c : _components)
        c->worldTicked(w, t);
}
