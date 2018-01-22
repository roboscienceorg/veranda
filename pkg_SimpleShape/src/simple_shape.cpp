#include "simple_shape.h"

#include <QDebug>
#include <cmath>

Simple_Shape::Simple_Shape(QObject *parent) : WorldObjectComponent_If(parent)
{
    shape_model = new Model({}, {}, this);
}

WorldObjectComponent_If *Simple_Shape::clone(QObject *newParent)
{
    Simple_Shape* out = new Simple_Shape(newParent);

    out->x.set(x.get());
    out->y.set(y.get());
    out->radius.set(radius.get());

    return out;
}

void Simple_Shape::_channelChanged(QVariant)
{
    if(_connected)
    {
        disconnectChannels();
        connectChannels();
    }
}

void Simple_Shape::_buildModels()
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

QVector<b2Body*> Simple_Shape::generateBodies(b2World *world, object_id oId, b2Body *anchor)
{
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(anchor->GetPosition().x + x.get().toFloat(), anchor->GetPosition().y + y.get().toFloat());
    bodyDef.angle = 0;
    body = world->CreateBody(&bodyDef);

    b2CircleShape circleShape;
    circleShape.m_radius = radius.get().toFloat();
    b2FixtureDef circleFixtureDef;
    circleFixtureDef.shape = &circleShape;
    circleFixtureDef.density = 1;
    circleFixtureDef.filter.groupIndex = -oId;
    body->CreateFixture(&circleFixtureDef);

    b2WeldJointDef jointDef;
    jointDef.bodyA = body;
    jointDef.bodyB = anchor;
    jointDef.collideConnected = false;

    joint = (b2WeldJoint*)world->CreateJoint(&jointDef);

    _buildModels();

    massChanged(this, body->GetMass());
    return {body};
}

void Simple_Shape::clearBodies(b2World* world)
{
    world->DestroyJoint(joint);
    world->DestroyBody(body);
    massChanged(this, 0);
}

void Simple_Shape::worldTicked(const b2World*, const double)
{
    double x = body->GetWorldCenter().x;
    double y = body->GetWorldCenter().y;
    double t = body->GetAngle();
    shape_model->setTransform(x, y, t*RAD2DEG);
}
