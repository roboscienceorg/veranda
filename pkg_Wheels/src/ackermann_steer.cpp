#include "ackermann_steer.h"
#include "basic_wheel.h"

#include <QDebug>
#include <cmath>
#include <functional>

Ackermann_Steer::Ackermann_Steer(QObject *parent) : WorldObjectComponent_If(parent)
{
    qRegisterMetaType<std_msgs::msg::Float32::SharedPtr>("std_msgs::msg::Float32::SharedPtr");

    //Update channel in if name or driven status changes
    connect(&_inputChannel, &Property::valueSet, this, &Ackermann_Steer::_refreshChannel);

    //If the axle width changes, we have to move and re-joint the wheels
    connect(&_l1, &Property::valueSet, this, &Ackermann_Steer::_jointWheels);

    //Update box2d interface
    connect(&_wradius, &Property::valueSet, this, &Ackermann_Steer::_attachWheelFixture);
    connect(&_wwidth, &Property::valueSet, this, &Ackermann_Steer::_attachWheelFixture);

    //Update drawn model
    connect(&_wradius, &Property::valueSet, this, &Ackermann_Steer::_buildModels);
    connect(&_wwidth, &Property::valueSet, this, &Ackermann_Steer::_buildModels);
    connect(&_l1, &Property::valueSet, this, &Ackermann_Steer::_buildModels);

    connect(&_density, &Property::valueSet,
    [=](QVariant d)
    {
        if(_world)
        {
            _lWheelFix->SetDensity(d.toDouble());
            _lWheelBody->ResetMassData();

            _rWheelFix->SetDensity(d.toDouble());
            _rWheelBody->ResetMassData();

            massChanged(this, _lWheelBody->GetMass() + _rWheelBody->GetMass() + _cBody->GetMass());
        }
    });

    //Signal slot connection to correctly thread incomming messages
    connect(this, &Ackermann_Steer::_receiveMessage, this, &Ackermann_Steer::_processMessage);

    _wheelModel = new Model({}, {}, this);
    _lWheelModel = new Model({}, {}, this);
    _rWheelModel = new Model({}, {}, this);
    _debugModel = new Model({}, {}, this);

    _wheelModel->addChildren({_lWheelModel, _rWheelModel/*, _debugModel*/});
}

void Ackermann_Steer::generateBodies(b2World* world, object_id oId, b2Body* anchor)
{
    clearBodies(world);

    b2BodyDef bDef;
    bDef.type = b2_dynamicBody;
    _lWheelBody = world->CreateBody(&bDef);
    _rWheelBody = world->CreateBody(&bDef);
    _cBody = world->CreateBody(&bDef);

    _anchor = anchor;
    _world = world;

    _jointWheels();

    _objectId = oId;

    _attachWheelFixture();
    _buildModels();
}

void Ackermann_Steer::_jointWheels()
{
    if(_world)
    {
        if(_lRevJoint)
        {
            _world->DestroyJoint(_lRevJoint);
            _world->DestroyJoint(_rRevJoint);
            _world->DestroyJoint(_cJoint);
        }

        //Leverage the move body function rather than do math manually
        moveBodyToLocalSpaceOfOtherBody(_cBody, _anchor, _xLocal.get().toDouble(), _yLocal.get().toDouble(), _thetaLocal.get().toDouble());
        moveBodyToLocalSpaceOfOtherBody(_rWheelBody, _cBody, _l1.get().toDouble()/2.0, 0, 90);
        moveBodyToLocalSpaceOfOtherBody(_lWheelBody, _cBody, -_l1.get().toDouble()/2.0, 0, 90);

        //qDebug() << "Main body:" << _cBody->GetWorldCenter().x << _cBody->GetWorldCenter().y << _cBody->GetAngle();
        //qDebug() << "Left body:" << _lWheelBody->GetWorldCenter().x << _lWheelBody->GetWorldCenter().y << _lWheelBody->GetAngle();
        //qDebug() << "Right body:" << _rWheelBody->GetWorldCenter().x << _rWheelBody->GetWorldCenter().y << _rWheelBody->GetAngle();

        b2RevoluteJointDef revDef;
        revDef.enableMotor = false;
        revDef.lowerAngle = revDef.upperAngle = 0;
        revDef.enableLimit = true;
        revDef.collideConnected = false;

        auto anchorPt = _lWheelBody->GetWorldCenter();
        revDef.bodyA = _anchor;
        revDef.bodyB = _lWheelBody;
        revDef.localAnchorA = _anchor->GetLocalPoint(anchorPt);
        revDef.localAnchorB = _lWheelBody->GetLocalPoint(anchorPt);
        revDef.referenceAngle = _lWheelBody->GetAngle() - _anchor->GetAngle();
        _lRevJoint = _world->CreateJoint(&revDef);

        anchorPt = _rWheelBody->GetWorldCenter();
        revDef.bodyA = _anchor;
        revDef.bodyB = _rWheelBody;
        revDef.localAnchorA = _anchor->GetLocalPoint(anchorPt);
        revDef.localAnchorB = _rWheelBody->GetLocalPoint(anchorPt);
        revDef.referenceAngle = _rWheelBody->GetAngle() - _anchor->GetAngle();
        _rRevJoint = _world->CreateJoint(&revDef);

        b2WeldJointDef weldDef;
        weldDef.collideConnected = false;
        anchorPt = _anchor->GetWorldCenter();
        weldDef.bodyA = _anchor;
        weldDef.bodyB = _cBody;
        weldDef.localAnchorA = _anchor->GetLocalPoint(anchorPt);
        weldDef.localAnchorB = _cBody->GetLocalPoint(anchorPt);
        weldDef.referenceAngle = _cBody->GetAngle() - _anchor->GetAngle();
        _cJoint = _world->CreateJoint(&weldDef);
    }
}

void Ackermann_Steer::clearBodies(b2World *world)
{
    if(_world)
    {
        world->DestroyJoint(_lRevJoint);
        world->DestroyBody(_lWheelBody);

        _lRevJoint = nullptr;
        _lWheelBody = nullptr;
        _lWheelFix = nullptr;

        world->DestroyJoint(_rRevJoint);
        world->DestroyBody(_rWheelBody);

        _rRevJoint = nullptr;
        _rWheelBody = nullptr;
        _rWheelFix = nullptr;

        world->DestroyJoint(_cJoint);
        world->DestroyBody(_cBody);

        _cJoint = nullptr;
        _cBody = nullptr;
        _cFix = nullptr;
    }

    _anchor = nullptr;
    _world = nullptr;

    massChanged(this, 0);
}

void Ackermann_Steer::_attachWheelFixture()
{
    //Can only add fixture if body defined
    if(_world)
    {
        //If old fixture, remove it
        if(_cFix)
        {
            _cBody->DestroyFixture(_cFix);
            _cFix = nullptr;

            _lWheelBody->DestroyFixture(_lWheelFix);
            _lWheelFix = nullptr;

            _rWheelBody->DestroyFixture(_rWheelFix);
            _rWheelFix = nullptr;
        }

        //Create rectangle to represent the wheel from top down
        b2FixtureDef fixDef;
        b2Shape* sh = Basic_Wheel::makeWheelShape(_wradius.get().toDouble(), _wwidth.get().toDouble());

        fixDef.isSensor = false;
        fixDef.shape = sh;
        fixDef.filter.groupIndex = -_objectId;
        fixDef.density = _density.get().toDouble();

        _lWheelFix = _lWheelBody->CreateFixture(&fixDef);
        _rWheelFix = _rWheelBody->CreateFixture(&fixDef);

        b2CircleShape* circ = new b2CircleShape;
        circ->m_radius = 1;
        _debugModel->addShapes({circ});

        fixDef.shape = circ;
        fixDef.density = 1;
        fixDef.isSensor = true;

        _cFix = _cBody->CreateFixture(&fixDef);

        massChanged(this, _lWheelBody->GetMass() + _rWheelBody->GetMass());

        delete sh;
    }
}

WorldObjectComponent_If *Ackermann_Steer::clone(QObject *newParent)
{
    Ackermann_Steer* out = new Ackermann_Steer(newParent);

    for(QString s : _properties.keys())
    {
        out->_properties[s].set(_properties[s].get(), true);
    }

    return out;
}

void Ackermann_Steer::setROSNode(std::shared_ptr<rclcpp::Node> node)
{
    _receiveChannel.reset();
    _rosNode = node;
}

void Ackermann_Steer::_refreshChannel(QVariant)
{
    disconnectChannels();
    connectChannels();
}

void Ackermann_Steer::connectChannels()
{
    if(_connected)
        disconnectChannels();

    //Only listen when the wheel is driven
    if(_rosNode)
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

void Ackermann_Steer::disconnectChannels()
{
    _receiveChannel.reset();
    _connected = false;
}

void Ackermann_Steer::_buildModels()
{
    qDeleteAll(_wheelModel->shapes());
    _wheelModel->removeShapes(_wheelModel->shapes());

    qDeleteAll(_lWheelModel->shapes());
    _lWheelModel->removeShapes(_lWheelModel->shapes());

    qDeleteAll(_rWheelModel->shapes());
    _rWheelModel->removeShapes(_rWheelModel->shapes());

    _wheelModel->removeChildren({_lWheelModel, _rWheelModel});

    delete _lWheelModel;
    delete _rWheelModel;

    _lWheelModel = Basic_Wheel::makeWheelModel(_wradius.get().toDouble(), _wwidth.get().toDouble());
    _rWheelModel = Basic_Wheel::makeWheelModel(_wradius.get().toDouble(), _wwidth.get().toDouble());

    _lWheelModel->setParent(this);
    _rWheelModel->setParent(this);

    _wheelModel->addChildren({_lWheelModel, _rWheelModel});

    b2EdgeShape* line = new b2EdgeShape;
    line->m_vertex1 = b2Vec2(-_l1.get().toDouble()/2.0, 0);
    line->m_vertex2 = b2Vec2(_l1.get().toDouble()/2.0, 0);

    _wheelModel->addShapes({line});

    _updateModelLocations();
}

void Ackermann_Steer::_updateModelLocations()
{
    if(_world)
    {
        double t = _cBody->GetAngle();
        double x = _cBody->GetWorldCenter().x;
        double y = _cBody->GetWorldCenter().y;
        _wheelModel->setTransform(x, y, t * RAD2DEG);

        _lWheelModel->setTransform(-_l1.get().toDouble()/2.0, 0, (_lWheelBody->GetAngle() - t) * RAD2DEG);
        _rWheelModel->setTransform(_l1.get().toDouble()/2.0, 0, (_rWheelBody->GetAngle() - t) * RAD2DEG);
    }
}

void Ackermann_Steer::worldTicked(const b2World*, const double)
{
    if(_lWheelBody && _rWheelBody)
    {
        Basic_Wheel::applyNoSlideConstraint(_lWheelBody, _wradius.get().toDouble());
        Basic_Wheel::applyNoSlideConstraint(_rWheelBody, _wradius.get().toDouble());

        _updateModelLocations();
    }
}

void Ackermann_Steer::_processMessage(const std_msgs::msg::Float32::SharedPtr data)
{
    const std::function<double(const double&)> acot = [](const double& x){return PI/2 - atan(x);};

    double targetAngle = std::min(PI/2.0, std::max(-PI/2.0, (double)data->data));
    _steerAngle.set(targetAngle);

    //qDebug() << "Steer angle:" << targetAngle;

    double targetLeft = 0, targetRight = 0;
    if(targetAngle < -0.001 || targetAngle > 0.001)
    {
        double cot = cos(targetAngle)/sin(targetAngle);
        double l1l2 = _l1.get().toDouble()*0.5/_l2.get().toDouble();

        double dtr = acot(cot - l1l2);
        double dtl = acot(cot + l1l2);

        targetRight = dtl;
        targetLeft = dtr;

        if(targetRight > PI/2) targetRight -= PI;
        if(targetLeft > PI/2) targetLeft -= PI;
    }

    ((b2RevoluteJoint*)_lRevJoint)->SetLimits(targetLeft, targetLeft);
    ((b2RevoluteJoint*)_rRevJoint)->SetLimits(targetRight, targetRight);

}