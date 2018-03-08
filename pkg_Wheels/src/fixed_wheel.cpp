#include "fixed_wheel.h"
#include "basic_wheel.h"

#include <QDebug>
#include <cmath>

Fixed_Wheel::Fixed_Wheel(QObject *parent) : WorldObjectComponent("Fixed Wheel", parent)
{
    qRegisterMetaType<std_msgs::msg::Float32::SharedPtr>("std_msgs::msg::Float32::SharedPtr");

    //Update channel in if name or driven status changes
    connect(&_inputChannel, &Property::valueSet, this, &Fixed_Wheel::_refreshChannel);
    connect(&_driven, &Property::valueSet, this, &Fixed_Wheel::_refreshChannel);

    //Update box2d interface
    connect(&_radius, &Property::valueSet, this, &Fixed_Wheel::_attachWheelFixture);
    connect(&_width, &Property::valueSet, this, &Fixed_Wheel::_attachWheelFixture);

    //Update drawn model
    connect(&_radius, &Property::valueSet, this, &Fixed_Wheel::_buildModels);
    connect(&_width, &Property::valueSet, this, &Fixed_Wheel::_buildModels);

    connect(&_density, &Property::valueSet,
    [=](QVariant d)
    {
        if(_wheelFix)
        {
            _wheelFix->SetDensity(d.toDouble());
        }
    });

    //Signal slot connection to correctly thread incomming messages
    connect(this, &Fixed_Wheel::_receiveMessage, this, &Fixed_Wheel::_processMessage);

    _wheelModel = new Model({}, {}, this);
    registerModel(_wheelModel);

    _buildModels();
}

void Fixed_Wheel::generateBodies(b2World* world, object_id oId, b2Body* anchor)
{
    clearBodies();
    _world = world;

    b2BodyDef bDef;
    bDef.type = b2_dynamicBody;
    _wheelBody = world->CreateBody(&bDef);
    registerBody(_wheelBody, {_wheelModel});

    b2WeldJointDef weldDef;
    auto anchorPt = anchor->GetWorldCenter();
    weldDef.bodyA = anchor;
    weldDef.bodyB = _wheelBody;
    weldDef.localAnchorA = anchor->GetLocalPoint(anchorPt);
    weldDef.localAnchorB = _wheelBody->GetLocalPoint(anchorPt);
    weldDef.referenceAngle = _wheelBody->GetAngle() - anchor->GetAngle();
    _weldJoint = world->CreateJoint(&weldDef);

    _objectId = oId;

    _attachWheelFixture();
}

void Fixed_Wheel::clearBodies()
{
    if(_world)
    {
        _world->DestroyJoint(_weldJoint);
        _world->DestroyBody(_wheelBody);
        unregisterBody(_wheelBody);
        _weldJoint = nullptr;
        _wheelBody = nullptr;
        _wheelFix = nullptr;
    }
    _world = nullptr;
}

void Fixed_Wheel::_attachWheelFixture()
{
    //Can only add fixture if body defined
    if(_wheelBody)
    {
        //If old fixture, remove it
        if(_wheelFix)
        {
            _wheelBody->DestroyFixture(_wheelFix);
            _wheelFix = nullptr;
        }

        //Create rectangle to represent the wheel from top down
        b2FixtureDef fixDef;
        b2Shape* wheel = Basic_Wheel::makeWheelShape(_radius.get().toDouble(), _width.get().toDouble());

        fixDef.isSensor = false;
        fixDef.shape = wheel;
        fixDef.filter.groupIndex = -_objectId;
        fixDef.density = _density.get().toDouble();

        _wheelFix = _wheelBody->CreateFixture(&fixDef);

        delete wheel;
    }
}

WorldObjectComponent *Fixed_Wheel::_clone(QObject *newParent)
{
    Fixed_Wheel* out = new Fixed_Wheel(newParent);

    return out;
}

void Fixed_Wheel::setROSNode(std::shared_ptr<rclcpp::Node> node)
{
    _receiveChannel.reset();
    _rosNode = node;
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
    if(_driven.get().toBool() && _rosNode)
    {
        QString inputChannel = _inputChannel.get().toString();

        if(inputChannel.size())
        {
            auto callback =
            [this](const std_msgs::msg::Float32::SharedPtr msg) -> void
            {
                _receiveMessage(msg);
            };
            _receiveChannel = _rosNode->create_subscription<std_msgs::msg::Float32>(inputChannel.toStdString(), callback);
        }
    }
    _connected = true;
}

void Fixed_Wheel::disconnectChannels()
{
    _receiveChannel.reset();
    _connected = false;
}

void Fixed_Wheel::_buildModels()
{
    _wheelModel->removeShapes(_wheelShapes);
    qDeleteAll(_wheelShapes);
    _wheelShapes.clear();

    Model* newModel = Basic_Wheel::makeWheelModel(_radius.get().toDouble(), _width.get().toDouble());

    _wheelShapes = newModel->shapes();
    _wheelModel->addShapes(_wheelShapes);

    delete newModel;
}

void Fixed_Wheel::_worldTicked(const double)
{
    if(_wheelBody)
    {
        Basic_Wheel::applyNoSlideConstraint(_wheelBody, _radius.get().toDouble());
        Basic_Wheel::applyNoSlipConstraint(_wheelBody, _radius.get().toDouble(), _targetAngularVelocity);
    }
}

void Fixed_Wheel::_processMessage(const std_msgs::msg::Float32::SharedPtr data)
{
    _targetAngularVelocity = data->data;
}
