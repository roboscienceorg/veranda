#include "sdsmt_simulator/world_object.h"

#include <QSet>

WorldObject::WorldObject(QVector<WorldObjectComponent *> components, QString name, QObject *parent) : WorldObjectComponent(name, parent), _components(components)
{
    QMap<QString, int> groupcounts;

    //Quick tally of if any components use the same group name
    for(WorldObjectComponent* c : _components)
    {
        groupcounts[c->getName()]++;
    }

    QSet<QString> multiples;
    for(auto iter = groupcounts.begin(); iter != groupcounts.end(); iter++)
        if(iter.value() > 1) multiples.insert(iter.key());

    //Initialize all aggregates
    for(WorldObjectComponent* c : _components)
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
        QMap<QString, QSharedPointer<PropertyView>> props = c->getProperties();

        //If more than 1 child uses group, append a number
        //to the group
        QString group = c->getName();

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
        for(Model* m : c->getModels())
            registerModel(m);

       /*************
        * Channels
        *************/
        if(c->usesChannels()) _useChannels = true;

        //Connect the children to our global transform
        //and initialize their local transform relative to us
        //Since this is construction, our transform is 0, 0, 0
        connect(this, &WorldObjectComponent::transformChanged, c, &WorldObjectComponent::setParentTransform);
        c->setParentTransform(QTransform(), false);
    }

    debugModel = new Model();
    registerModel(debugModel);
}

WorldObject* WorldObject::_clone(QObject *newParent)
{
    QVector<WorldObjectComponent*> childClones;
    for(WorldObjectComponent* c : _components)
        childClones.push_back(c->_clone());

    WorldObject* copy = new WorldObject(childClones, getName(), newParent);
    return copy;
}

void WorldObject::clearBodies()
{
    if(!_world) return;

    QVector<WorldObjectComponent*> components = getComponents();

    if(anchorBody)
    {
        for(WorldObjectComponent* c : components)
            c->clearBodies();

        unregisterBody(anchorBody);
        _world->DestroyBody(anchorBody);
        anchorBody = nullptr;
    }
    _world = nullptr;
}

void WorldObject::generateBodies(b2World* world, object_id oId, b2Body* anchorBody)
{
    clearBodies();
    _world = world;

    //qDebug() << "Creating object at " << _locX.get().toDouble() << ", " << _locY.get().toDouble() << " : " << _locTheta.get().toDouble();

    QVector<WorldObjectComponent*> components = getComponents();

    b2BodyDef anchorDef;
    anchorDef.type = b2_dynamicBody;
    anchorBody = world->CreateBody(&anchorDef);
    registerBody(anchorBody, {debugModel}, true);

    b2CircleShape* circ = new b2CircleShape;
    circ->m_p = b2Vec2(0, 0);
    circ->m_radius = 1;

    b2FixtureDef fix;
    fix.shape = circ;
    fix.density = 1;
    fix.isSensor = true;
    anchorBody->CreateFixture(&fix);

    debugModel->addShapes(QVector<b2Shape*>{circ});

    for(int i = 0; i < components.size(); i++)
        components[i]->generateBodies(world, oId, anchorBody);
}

void WorldObject::connectChannels()
{
    if(_useChannels)
    {
        for(WorldObjectComponent* c : _components)
            c->connectChannels();
    }
}

void WorldObject::disconnectChannels()
{
    if(_useChannels)
    {
        for(WorldObjectComponent* c : _components)
            c->disconnectChannels();
    }
}

void WorldObject::_worldTicked(const double t)
{
    for(WorldObjectComponent* c : _components)
        c->worldTicked(t);
}

void WorldObject::_syncModels()
{
    for(WorldObjectComponent* c : _components)
        c->syncModels();
}

void WorldObject::setROSNode(std::shared_ptr<rclcpp::Node> node)
{
    for(WorldObjectComponent* c : _components)
        c->setROSNode(node);
}
