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

    _debugModel = new Model();
    registerModel(_debugModel);
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

    for(WorldObjectComponent* c : _components)
        c->clearBodies();

    unregisterBody(_anchorBody);
    _world->DestroyBody(_anchorBody);
    _anchorBody = nullptr;

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

    for(int i = 0; i < components.size(); i++)
        components[i]->generateBodies(world, oId, _anchorBody);
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

void WorldObject::readJson(const QJsonObject &json)
{
    if (json.contains("properties") && json["properties"].isArray())
    {
        QJsonArray propertyArray = json["properties"].toArray();
        for (int i = 0; i < propertyArray.size(); i++)
        {
            QJsonObject propertyObject = propertyArray[i].toObject();
            if (propertyObject.contains("key") && propertyObject["key"].isObject()
             && propertyObject.contains("value") && propertyObject["value"].isObject())
            {
                PropertyView v;
                v.set(propertyObject["value"].toString(), true);
                QSharedPointer p(&v);
                _properties[propertyObject["key"].toString()] = p;
            }
        }
    }

    if (json.contains("components") && json["components"].isArray())
    {
        QJsonArray componentArray = json["components"].toArray();
        for (int i = 0; i < componentArray.size(); i++)
        {
            QJsonObject componentObject = componentArray[i].toObject();
            if (componentObject.contains("pluginName") && componentObject["pluginName"].isObject() && plugins.contains(componentObject["pluginName"]))
            {
                WorldObjectComponent* comp = plugins[componentObject["pluginName"]]->createComponent();
                QMap<QString, PropertyView> props = comp->getProperties();

                if (componentObject.contains("properties") && componentObject["properties"].isArray())
                {
                    QJSonArray propArray = componentObject["properties"].toArray();
                    for (int j = 0; j < propArray.size(); j++)
                    {
                        QJsonObject propertyObject = propArray[j].toObject();
                        if (propertyObject.contains("key") && componenetObject["key"].isObject()
                         && propertyObject.contains("value") && componenetObject["value"].isObject())
                        {
                            props[propertyObject["key"]].set(propertyObject["value"], true);
                        }
                    }
                }
                _components.push_back(comp);
            }
        }
    }
}

void WorldObject::writeJson(QJsonObject &json) const
{
    QJsonArray propArray;
    for (QMap::iterator it = _properties.begin();it != _properties.end(); it++)
    {
        QJsonObject propObj;
        propObj["key"] = it.key();
        propObj["value"] = it.value().get();
        propArray.append(propObj);
    }
    json["properties"] = propArray;

    QJsonArray compArray;
    foreach (const WorldObjectComponent comp, _components)
    {
        QJsonObject compObject;
        comp.writeJson(compObject);
        compArray.append(compObject);
    }
    json["components"] = compArray;
}
