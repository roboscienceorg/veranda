#include "circle.h"

#include <QDebug>
#include <cmath>

Circle::Circle(QObject *parent) : WorldObjectComponent_If(parent)
{
    shape_model = new Model({}, {}, this);
}

WorldObjectComponent_If *Circle::clone(QObject *newParent)
{
    Circle* out = new Circle(newParent);

    for(QString s : _properties.keys())
    {
        out->_properties[s].set(_properties[s].get(), true);
    }

    return out;
}

void Circle::_channelChanged(QVariant)
{
    if(_connected)
    {
        disconnectChannels();
        connectChannels();
    }
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

QVector<b2Body*> Circle::generateBodies(b2World *world, object_id oId, b2Body *anchor)
{
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.angle = 0;
    body = world->CreateBody(&bodyDef);

    moveBodyToLocalSpaceOfOtherBody(body, anchor, x.get().toFloat(), y.get().toFloat());

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

void Circle::clearBodies(b2World* world)
{
    world->DestroyJoint(joint);
    world->DestroyBody(body);
    massChanged(this, 0);
}

void Circle::worldTicked(const b2World*, const double)
{
    double x = body->GetWorldCenter().x;
    double y = body->GetWorldCenter().y;
    double t = body->GetAngle();
    shape_model->setTransform(x, y, t*RAD2DEG);
}
