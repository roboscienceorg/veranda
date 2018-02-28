#include "rectangle.h"

#include <QDebug>
#include <cmath>

Rectangle::Rectangle(QObject *parent) : WorldObjectComponent_If(parent)
{
    shape_model = new Model({}, {}, this);
}

WorldObjectComponent_If *Rectangle::clone(QObject *newParent)
{
    Rectangle* out = new Rectangle(newParent);

    for(QString s : _properties.keys())
    {
        out->_properties[s].set(_properties[s].get(), true);
    }

    return out;
}

void Rectangle::_channelChanged(QVariant)
{
    if(_connected)
    {
        disconnectChannels();
        connectChannels();
    }
}

void Rectangle::_buildModels()
{
    //Clear always-showing model
    QVector<b2Shape*> oldModel = shape_model->shapes();
    shape_model->removeShapes(oldModel);
    qDeleteAll(oldModel);

    //Build always-showing model...
    b2PolygonShape* rectangle = new b2PolygonShape;
    rectangle->SetAsBox(width.get().toFloat() / 2.0, height.get().toFloat() / 2.0,  b2Vec2(0,0), rotation.get().toFloat() * DEG2RAD);
    shape_model->addShapes({rectangle});
}

QVector<b2Body*> Rectangle::generateBodies(b2World *world, object_id oId, b2Body *anchor)
{
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.angle = 0;
    body = world->CreateBody(&bodyDef);

    moveBodyToLocalSpaceOfOtherBody(body, anchor, x.get().toFloat(), y.get().toFloat());

    b2PolygonShape rectangleShape;
    rectangleShape.SetAsBox(width.get().toFloat(), height.get().toFloat());
    b2FixtureDef rectangleFixtureDef;
    rectangleFixtureDef.shape = &rectangleShape;
    rectangleFixtureDef.density = 1;
    rectangleFixtureDef.filter.groupIndex = -oId;
    body->CreateFixture(&rectangleFixtureDef);

    b2WeldJointDef jointDef;
    jointDef.bodyA = body;
    jointDef.bodyB = anchor;
    jointDef.collideConnected = false;

    joint = (b2WeldJoint*)world->CreateJoint(&jointDef);

    _buildModels();

    massChanged(this, body->GetMass());

    return {body};
}

void Rectangle::clearBodies(b2World* world)
{
    world->DestroyJoint(joint);
    world->DestroyBody(body);
    massChanged(this, 0);
}

void Rectangle::worldTicked(const b2World*, const double)
{
    double x = body->GetWorldCenter().x;
    double y = body->GetWorldCenter().y;
    double t = body->GetAngle();
    shape_model->setTransform(x, y, t*RAD2DEG);
}
