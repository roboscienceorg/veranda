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
    connect(&max_force, &Property::valueSet, this, &Fixed_Wheel::_updateForce);

    //Signal slot connection to correctly thread incomming messages
    connect(this, &Fixed_Wheel::_receiveMessage, this, &Fixed_Wheel::_processMessage);

    wheel_model = new Model({}, {}, this);
}

void Fixed_Wheel::generateBodies(b2World* world, object_id oId, b2Body* anchor)
{

}

WorldObjectComponent_If *Fixed_Wheel::clone(QObject *newParent)
{
    Fixed_Wheel* out = new Fixed_Wheel(newParent);

    out->input_channel.set(input_channel.get());
    out->radius.set(radius.get());
    out->width.set(width.get());
    out->max_force.set(max_force.get());
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

void Fixed_Wheel::_attachWheelFixture()
{
    //Can only add fixture if body defined
    if(wheelBody)
    {

    }
}

void Fixed_Wheel::_buildModels()
{
    b2PolygonShape* sh = new b2PolygonShape;
    sh->SetAsBox(width.get().toDouble(), radius.get().toDouble()*2,
                 b2Vec2(x_local.get().toDouble(), y_local.get().toDouble()), theta_local.get().toDouble()*DEG2RAD);

    wheel_model->removeShapes(QVector<b2Shape*>{wheel_shape});
    wheel_model->addShapes(QVector<b2Shape*>{sh});
    delete wheel_shape;
    wheel_shape = sh;
}

void Fixed_Wheel::worldTicked(const b2World*, const double&)
{

}

void Fixed_Wheel::_processMessage(std_msgs::Float32 data)
{

}
