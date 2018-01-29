#include "fixed_wheel.h"

#include <QDebug>
#include <cmath>

Fixed_Wheel::Fixed_Wheel(QObject *parent) : WorldObjectComponent_If(parent)
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
        if(_wheelFix && _wheelBody)
        {
            _wheelFix->SetDensity(d.toDouble());
            _wheelBody->ResetMassData();
            massChanged(this, _wheelBody->GetMass());
        }
    });

    //Signal slot connection to correctly thread incomming messages
    connect(this, &Fixed_Wheel::_receiveMessage, this, &Fixed_Wheel::_processMessage);

    _wheelModel = new Model({}, {}, this);
}

void Fixed_Wheel::generateBodies(b2World* world, object_id oId, b2Body* anchor)
{
    clearBodies(world);

    b2BodyDef bDef;
    bDef.type = b2_dynamicBody;
    _wheelBody = world->CreateBody(&bDef);

    moveBodyToLocalSpaceOfOtherBody(_wheelBody, anchor, _xLocal.get().toDouble(), _yLocal.get().toDouble(), _thetaLocal.get().toDouble());

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
    _buildModels();
}

void Fixed_Wheel::clearBodies(b2World *world)
{
    if(nullptr != _wheelBody)
    {
        world->DestroyJoint(_weldJoint);
        world->DestroyBody(_wheelBody);

        _weldJoint = nullptr;
        _wheelBody = nullptr;
        _wheelFix = nullptr;

        massChanged(this, 0);
    }
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
        b2PolygonShape wheel;

        wheel.SetAsBox(_radius.get().toDouble(), _width.get().toDouble()/2.0);

        fixDef.isSensor = false;
        fixDef.shape = &wheel;
        fixDef.filter.groupIndex = -_objectId;
        fixDef.density = _density.get().toDouble();

        _wheelFix = _wheelBody->CreateFixture(&fixDef);

        _localWheelFrontUnit = b2Vec2(1, 0);
        _localWheelRightUnit = b2Vec2(0, 1);

        massChanged(this, _wheelBody->GetMass());
    }
}

WorldObjectComponent_If *Fixed_Wheel::clone(QObject *newParent)
{
    Fixed_Wheel* out = new Fixed_Wheel(newParent);

    out->_inputChannel.set(_inputChannel.get());
    out->_radius.set(_radius.get());
    out->_width.set(_width.get());
    out->_driven.set(_driven.get());
    out->_xLocal.set(_xLocal.get());
    out->_yLocal.set(_yLocal.get());
    out->_thetaLocal.set(_thetaLocal.get());

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
            _connected = true;
        }
    }
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

    b2PolygonShape* sh = new b2PolygonShape;
    sh->SetAsBox(_radius.get().toDouble(), _width.get().toDouble()/2.0);

    b2EdgeShape* line = new b2EdgeShape;
    line->m_vertex1 = b2Vec2(0, 0);
    line->m_vertex2 = b2Vec2(_radius.get().toDouble(), 0);

    _wheelShapes = QVector<b2Shape*>{sh, line};
    _wheelModel->addShapes(_wheelShapes);
}

void Fixed_Wheel::worldTicked(const b2World*, const double)
{
    if(_wheelBody)
    {
        b2Vec2 front = _wheelBody->GetWorldVector(_localWheelFrontUnit);
        b2Vec2 right = _wheelBody->GetWorldVector(_localWheelRightUnit);

        //Calculation to find target velocity from
        //rotations per second; original idea was to behave more line
        //real moter controller
        //Circumference * ratio of 2PI to radians traveled
        double targetVelocity = 2*PI*_radius.get().toDouble() * (_targetAngularVelocity/(2*PI));

        //Calculate lateral movement
        b2Vec2 lateralVelocity = right * b2Dot( right, _wheelBody->GetLinearVelocity() );

        //Calculate linear movement
        b2Vec2 forwardVelocity = front * b2Dot( front, _wheelBody->GetLinearVelocity() );

        //Negate slip/slide
        b2Vec2 impulse1 = -lateralVelocity * _wheelBody->GetMass();
        b2Vec2 impulse2 = (front * targetVelocity - forwardVelocity) * _wheelBody->GetMass();
        _wheelBody->ApplyLinearImpulse( impulse1 + impulse2, _wheelBody->GetWorldCenter(), true );

        double x = _wheelBody->GetWorldCenter().x;
        double y = _wheelBody->GetWorldCenter().y;
        double t = _wheelBody->GetAngle();
        _wheelModel->setTransform(x, y, t*RAD2DEG);
    }
}

void Fixed_Wheel::_processMessage(const std_msgs::msg::Float32::SharedPtr data)
{
    _targetAngularVelocity = data->data;
}
