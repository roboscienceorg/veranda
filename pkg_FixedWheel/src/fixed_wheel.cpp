#include "fixed_wheel.h"
#include "std_msgs/ByteMultiArray.h"

#include <QDebug>
#include <cmath>

Fixed_Wheel::Fixed_Wheel(QObject *parent) : WorldObjectComponent_If(parent)
{
    qRegisterMetaType<std_msgs::Float32>("std_msgs::Float32");

    //Update channel in if name or driven status changes
    connect(&input_channel, &Property::valueSet, this, &Fixed_Wheel::_refreshChannel);
    connect(&driven, &Property::valueSet, this, &Fixed_Wheel::_refreshChannel);

    //Update box2d interface
    connect(&radius, &Property::valueSet, this, &Fixed_Wheel::_attachWheelFixture);
    connect(&width, &Property::valueSet, this, &Fixed_Wheel::_attachWheelFixture);

    //Update drawn model
    connect(&radius, &Property::valueSet, this, &Fixed_Wheel::_buildModels);
    connect(&width, &Property::valueSet, this, &Fixed_Wheel::_buildModels);

    //Update current force when max force changes
    connect(&max_rps, &Property::valueSet, this, &Fixed_Wheel::_updateForce);

    //Signal slot connection to correctly thread incomming messages
    connect(this, &Fixed_Wheel::_receiveMessage, this, &Fixed_Wheel::_processMessage);

    wheel_model = new Model({}, {}, this);
}

void Fixed_Wheel::generateBodies(b2World* world, object_id oId, b2Body* anchor)
{
    clearBodies(world);

    b2BodyDef bDef;
    bDef.angle = theta_local.get().toDouble()*DEG2RAD;
    bDef.position = b2Vec2(x_local.get().toDouble(), y_local.get().toDouble());
    bDef.type = b2_dynamicBody;
    wheelBody = world->CreateBody(&bDef);

    b2WeldJointDef weldDef;
    auto anchorPt = anchor->GetWorldCenter();
    weldDef.bodyA = anchor;
    weldDef.bodyB = wheelBody;
    weldDef.localAnchorA = anchor->GetLocalPoint(anchorPt);
    weldDef.localAnchorB = wheelBody->GetLocalPoint(anchorPt);
    weldDef.referenceAngle = wheelBody->GetAngle() - anchor->GetAngle();
    weldJoint = world->CreateJoint(&weldDef);

    objectId = oId;

    _attachWheelFixture();
    _buildModels();
}

void Fixed_Wheel::clearBodies(b2World *world)
{
    if(nullptr != wheelBody)
    {
        world->DestroyJoint(weldJoint);
        world->DestroyBody(wheelBody);

        weldJoint = nullptr;
        wheelBody = nullptr;
        wheelFix = nullptr;
    }
}

void Fixed_Wheel::_attachWheelFixture()
{
    //Can only add fixture if body defined
    if(wheelBody)
    {
        //If old fixture, remove it
        if(wheelFix)
        {
            wheelBody->DestroyFixture(wheelFix);
            wheelFix = nullptr;
        }

        //Create rectangle to represent the wheel from top down
        b2FixtureDef fixDef;
        b2PolygonShape wheel;

        wheel.SetAsBox(radius.get().toDouble()*2, width.get().toDouble());

        fixDef.isSensor = false;
        fixDef.shape = &wheel;
        fixDef.filter.groupIndex = -objectId;

        wheelFix = wheelBody->CreateFixture(&fixDef);

        localWheelFrontUnit = b2Vec2(1, 0);
        localWheelRightUnit = b2Vec2(0, 1);
    }
}

WorldObjectComponent_If *Fixed_Wheel::clone(QObject *newParent)
{
    Fixed_Wheel* out = new Fixed_Wheel(newParent);

    out->input_channel.set(input_channel.get());
    out->radius.set(radius.get());
    out->width.set(width.get());
    out->max_rps.set(max_rps.get());
    out->driven.set(driven.get());
    out->x_local.set(x_local.get());
    out->y_local.set(y_local.get());
    out->theta_local.set(theta_local.get());

    return out;
}

void Fixed_Wheel::_refreshChannel(QVariant)
{
    disconnectChannels();
    connectChannels();
}

void Fixed_Wheel::connectChannels()
{
    if(_connected)
        disconnectChannels();

    //Only listen when the wheel is driven
    if(driven.get().toBool())
        _inputChannel = input_channel.get().toString();
        _receiveChannel = _rosNode.subscribe(_inputChannel.toStdString(), 5, &Fixed_Wheel::_receiveMessage, this);
        _connected = true;
}

void Fixed_Wheel::disconnectChannels()
{
    _receiveChannel.shutdown();
    _connected = false;
}

void Fixed_Wheel::_buildModels()
{
    b2PolygonShape* sh = new b2PolygonShape;
    sh->SetAsBox(radius.get().toDouble()*2, width.get().toDouble());

    wheel_model->removeShapes(QVector<b2Shape*>{wheel_shape});
    wheel_model->addShapes(QVector<b2Shape*>{sh});
    delete wheel_shape;
    wheel_shape = sh;
}

void Fixed_Wheel::worldTicked(const b2World*, const double)
{
    qDebug() << "Wheel at " << wheelBody->GetPosition().x << ", " << wheelBody->GetPosition().y;

    b2Vec2 front = wheelBody->GetWorldVector(localWheelFrontUnit);
    b2Vec2 right = wheelBody->GetWorldVector(localWheelRightUnit);

    double rps = max_rps.get().toDouble();
    qDebug() << "Target rps: " << rps;

    //Circumference * ratio of 2PI to radians traveled
    double linear_target = 2*PI*radius.get().toDouble() * (rps/(2*PI));
    qDebug() << "Target linear velocity: " << linear_target << " m/s";

    //Calculate lateral movement
    b2Vec2 lateralVelocity = right * b2Dot( right, wheelBody->GetLinearVelocity() );

    //Calculate linear movement
    b2Vec2 forwardVelocity = front * b2Dot( front, wheelBody->GetLinearVelocity() );

    qDebug() << "Current forward velocity: " << forwardVelocity.Length();
    qDebug() << "Current lateral velocity: " << lateralVelocity.Length();

    //Negate slip/slide
    b2Vec2 impulse = -lateralVelocity * wheelBody->GetMass();
    qDebug() << "Apply " << (-lateralVelocity).Length() << " lateral correction";
    wheelBody->ApplyLinearImpulse( impulse, wheelBody->GetWorldCenter(), true );

    impulse = front * (linear_target-forwardVelocity.Length()) * wheelBody->GetMass();
    qDebug() << "Apply " << (linear_target-forwardVelocity.Length()) << " forward correction";
    wheelBody->ApplyLinearImpulse( impulse, wheelBody->GetWorldCenter(), true );

    double x = wheelBody->GetWorldCenter().x;
    double y = wheelBody->GetWorldCenter().y;
    double t = wheelBody->GetAngle();
    wheel_model->setTransform(x, y, t*RAD2DEG);
}

void Fixed_Wheel::_processMessage(std_msgs::Float32 data)
{

}
