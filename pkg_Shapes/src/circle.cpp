#include "circle.h"

#include <QDebug>
#include <cmath>

Circle::Circle(QObject *parent) : WorldObjectComponent("Circle", parent)
{
    shape_model = new Model({}, {}, this);
    registerModel(shape_model);

    _buildModels();

    connect(&radius, &Property::valueSet, this, &Circle::_buildModels);
    connect(&radius, &Property::valueSet, this, &Circle::_makeFixtures);
}

WorldObjectComponent *Circle::clone(QObject *newParent)
{
    Circle* out = new Circle(newParent);

    for(QString s : _properties.keys())
    {
        out->_properties[s]->set(_properties[s]->get(), true);
    }

    return out;
}

void Circle::_buildModels()
{
    //Clear always-showing model
    QVector<b2Shape*> oldModel = shape_model->shapes();
    shape_model->removeShapes(oldModel);
    qDeleteAll(oldModel);

    //Build always-showing model...
    b2CircleShape* circle = new b2CircleShape;
    circle->m_radius = radius.get().toFloat();
    circle->m_p = b2Vec2(0,0);
    shape_model->addShapes({circle});
}

void Circle::generateBodies(b2World *world, object_id oId, b2Body *anchor)
{
    clearBodies();
    _world = world;
    _oid = oId;

    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    double x, y, t;
    getTransform(x, y, t);
    bodyDef.angle = t*DEG2RAD;
    bodyDef.position.Set(x, y);
    body = world->CreateBody(&bodyDef);
    registerBody(body, {shape_model});

    b2WeldJointDef weldDef;
    auto anchorPt = anchor->GetWorldCenter();
    weldDef.bodyA = anchor;
    weldDef.bodyB = body;
    weldDef.localAnchorA = anchor->GetLocalPoint(anchorPt);
    weldDef.localAnchorB = body->GetLocalPoint(anchorPt);
    weldDef.referenceAngle = body->GetAngle() - anchor->GetAngle();
    joint = (b2WeldJoint*) world->CreateJoint(&weldDef);

    _makeFixtures();
}

void Circle::_makeFixtures()
{
    if(body)
    {
        if(fixture)
            body->DestroyFixture(fixture);

        b2CircleShape circleShape;
        circleShape.m_radius = radius.get().toFloat();
        b2FixtureDef circleFixtureDef;
        circleFixtureDef.shape = &circleShape;
        circleFixtureDef.density = 1;
        circleFixtureDef.filter.groupIndex = -_oid;
        fixture = body->CreateFixture(&circleFixtureDef);
    }
}

void Circle::clearBodies()
{
    if(_world)
    {
        _world->DestroyJoint(joint);
        _world->DestroyBody(body);

        body = nullptr;
        fixture = nullptr;
        joint = nullptr;
    }
    _world = nullptr;
}
