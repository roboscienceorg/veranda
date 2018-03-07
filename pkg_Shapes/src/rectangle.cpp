#include "rectangle.h"

#include <QDebug>
#include <cmath>

Rectangle::Rectangle(QObject *parent) : WorldObjectComponent("Rectangle", parent)
{
    shape_model = new Model({}, {}, this);
    registerModel(shape_model);

    _buildModels();

    connect(&height, &Property::valueSet, this, &Rectangle::_buildModels);
    connect(&height, &Property::valueSet, this, &Rectangle::_makeFixtures);
    connect(&width, &Property::valueSet, this, &Rectangle::_buildModels);
    connect(&width, &Property::valueSet, this, &Rectangle::_makeFixtures);
}

WorldObjectComponent *Rectangle::clone(QObject *newParent)
{
    Rectangle* out = new Rectangle(newParent);

    for(QString s : _properties.keys())
    {
        out->_properties[s]->set(_properties[s]->get(), true);
    }

    return out;
}

void Rectangle::_buildModels()
{
    //Clear always-showing model
    QVector<b2Shape*> oldModel = shape_model->shapes();
    shape_model->removeShapes(oldModel);
    qDeleteAll(oldModel);

    //Build always-showing model...
    b2PolygonShape* rectangle = new b2PolygonShape;
    rectangle->SetAsBox(width.get().toFloat() / 2.0, height.get().toFloat() / 2.0);
    shape_model->addShapes({rectangle});
}

void Rectangle::generateBodies(b2World *world, object_id oId, b2Body *anchor)
{
    clearBodies();
    _world = world;

    _oid = oId;

    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    double x, y, t;
    getTransform(x, y, t);
    bodyDef.angle = t * DEG2RAD;
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

void Rectangle::_makeFixtures()
{
    if(body)
    {
        if(fixture)
            body->DestroyFixture(fixture);

        b2PolygonShape rectangleShape;
        rectangleShape.SetAsBox(width.get().toDouble(), height.get().toDouble());

        b2FixtureDef rectangleFixtureDef;
        rectangleFixtureDef.shape = &rectangleShape;
        rectangleFixtureDef.density = 1;
        rectangleFixtureDef.filter.groupIndex = -_oid;
        fixture = body->CreateFixture(&rectangleFixtureDef);
    }
}

void Rectangle::clearBodies()
{
    if(_world)
    {
        _world->DestroyJoint(joint);
        _world->DestroyBody(body);

        fixture = nullptr;
        body = nullptr;
        joint = nullptr;
    }
    _world = nullptr;
}
