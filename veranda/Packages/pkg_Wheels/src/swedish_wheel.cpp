#include "swedish_wheel.h"
#include "basic_wheel.h"

#include <QDebug>
#include <QSharedPointer>

#include <cmath>
#include <cassert>

Swedish_Wheel::Swedish_Wheel(QObject *parent) :
    WorldObjectComponent("Swedish Wheel", "Wheels", parent),
    drive_filter(QSharedPointer<PropertyView>(new PropertyView(&noise_mu)), QSharedPointer<PropertyView>(new PropertyView(&noise_sigma)), 1)
{
    qRegisterMetaType<std_msgs::msg::Float32::SharedPtr>("std_msgs::msg::Float32::SharedPtr");

    //Update channel in if name or driven status changes
    connect(&_inputChannel, &Property::valueSet, this, &Swedish_Wheel::_refreshChannel);
    connect(&_driven, &Property::valueSet, this, &Swedish_Wheel::_refreshChannel);

    //Update box2d interface
    connect(&_radius, &Property::valueSet, this, &Swedish_Wheel::_attachWheelFixture);
    connect(&_width, &Property::valueSet, this, &Swedish_Wheel::_attachWheelFixture);

    //Update drawn model
    connect(&_radius, &Property::valueSet, this, &Swedish_Wheel::_buildModels);
    connect(&_width, &Property::valueSet, this, &Swedish_Wheel::_buildModels);
    connect(&_rollerAngle, &Property::valueSet, this, &Swedish_Wheel::_buildModels);

    connect(&_density, &Property::valueSet,
    [=](QVariant d)
    {
        if(_wheelFix)
        {
            _wheelFix->SetDensity(d.toDouble());
            _wheelBody->ResetMassData();
        }
    });

    //Signal slot connection to correctly thread incomming messages
    connect(this, &Swedish_Wheel::_receiveMessage, this, &Swedish_Wheel::_processMessage);

    _wheelModel = new Model({}, {}, this);
    registerModel(_wheelModel);

    registerChild(&wheelEncoder);

    _buildModels();
}

void Swedish_Wheel::_generateBodies(b2World* world, object_id oId, b2Body* anchor)
{
    clearBodies();
    _world = world;

    b2BodyDef bDef;
    bDef.type = b2_dynamicBody;
    _wheelBody = world->CreateBody(&bDef);
    registerBody(_wheelBody, {_wheelModel}, true);

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

void Swedish_Wheel::_clearBodies()
{
    if(_world)
    {
        wheelEncoder.setWheel(nullptr, 0);

        _world->DestroyJoint(_weldJoint);
        _world->DestroyBody(_wheelBody);
        unregisterBody(_wheelBody);
        _weldJoint = nullptr;
        _wheelBody = nullptr;
        _wheelFix = nullptr;
    }
    _world = nullptr;
}

void Swedish_Wheel::_attachWheelFixture()
{
    //Can only add fixture if body defined
    if(_wheelBody)
    {
        wheelEncoder.setWheel(_wheelBody, _radius.get().toDouble());

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

WorldObjectComponent *Swedish_Wheel::_clone(QObject *newParent)
{
    Swedish_Wheel* out = new Swedish_Wheel(newParent);

    return out;
}

void Swedish_Wheel::_setROSNode(std::shared_ptr<rclcpp::Node> node)
{
    _receiveChannel.reset();
    _rosNode = node;
}

void Swedish_Wheel::_refreshChannel(QVariant)
{
    if(_receiveChannel)
    {
        connectChannels();
    }
}

void Swedish_Wheel::_connectChannels()
{
    //qDebug() << "Connecting Fixed Wheel Channels";
    disconnectChannels();

    //Only listen when the wheel is driven
    //qDebug() << _driven.get().toBool() << " && " << (bool)_rosNode;
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

            //qDebug() << "Channel: " << inputChannel << " created: " << _receiveChannel.get();
        }
    }
    //qDebug() << "";
}

void Swedish_Wheel::_disconnectChannels()
{
    //qDebug() << "Channel: " << _inputChannel.get().toString() << " deleted";
    _receiveChannel.reset();
}

QDebug operator << (QDebug dbg, const b2Vec2& pt)
{
    dbg.nospace() << "(" << pt.x << ", " << pt.y << ")";
    return dbg.maybeSpace();
}

QDebug operator << (QDebug dbg, const b2EdgeShape& line)
{
    dbg.nospace() << "{" << line.m_vertex1 << " -> " << line.m_vertex2 << "}";
    return dbg.maybeSpace();
}

void Swedish_Wheel::_buildModels()
{
    _wheelModel->removeShapes(_wheelShapes);
    qDeleteAll(_wheelShapes);
    _wheelShapes.clear();

    Model* newModel = Basic_Wheel::makeWheelModel(_radius.get().toDouble(), _width.get().toDouble());
    _wheelShapes = newModel->shapes();

    double quadrant_angle = std::abs(_rollerAngle.get().toDouble());
    if(quadrant_angle < 0.0001)
    {
        b2EdgeShape* line = new b2EdgeShape();
        line->m_vertex1 = b2Vec2(_radius.get().toDouble(), 0);
        line->m_vertex2 = b2Vec2(-_radius.get().toDouble(), 0);

        b2Vec2 right(0, 0.05);
        b2EdgeShape* frontLeft = new b2EdgeShape();
        frontLeft->m_vertex1 = line->m_vertex1 - right;
        frontLeft->m_vertex2 = -right;

        b2EdgeShape* frontRight = new b2EdgeShape();
        frontRight->m_vertex1 = line->m_vertex1 + right;
        frontRight->m_vertex2 = right;

        _wheelShapes << line << frontLeft << frontRight;
    }
    else
    {
        quadrant_angle *= DEG2RAD;
        double dx = cos(quadrant_angle);
        double dy = sin(quadrant_angle);
        double halfWidth = _width.get().toDouble() / 2.0;
        double radius = _radius.get().toDouble();

        b2Vec2 backCorner(-radius, halfWidth);
        b2Vec2 frontCorner(radius, -halfWidth);
        b2Vec2 diag = frontCorner - backCorner;
        double diagLen = diag.Length();
        diag /= diagLen;

        const int ROLLER_COUNT = 6;
        int rollers = ROLLER_COUNT;
        for(int i=rollers-1; i > 0; i--)
        {
            b2EdgeShape* line = new b2EdgeShape();
            b2Vec2 center = backCorner + diag * (diagLen/rollers) * i;

            line->m_vertex1.Set((dx/dy) * (halfWidth-center.y) + center.x, halfWidth);
            if(line->m_vertex1.x > radius)
                line->m_vertex1.Set(radius, center.y + (radius-center.x)*dy/dx);

            line->m_vertex2.Set((-dx/dy) * (center.y + halfWidth) + center.x, -halfWidth);
            if(line->m_vertex2.x < -radius)
                line->m_vertex2.Set(-radius, center.y + (center.x + radius)*-dy/dx);

            if(_rollerAngle.get().toDouble() < 0)
            {
                line->m_vertex1.x = -line->m_vertex1.x;
                line->m_vertex2.x = -line->m_vertex2.x;
            }

            _wheelShapes << line;
        }
    }

    _wheelModel->addShapes(_wheelShapes);

    delete newModel;
}

void Swedish_Wheel::_worldTicked(const double)
{
    if(_wheelBody)
    {
        if(_driven.get().toBool())
        {
            double targetTheta =  _targetAngularVelocity + drive_filter.apply();
            double targetLinear = _radius.get().toDouble() * targetTheta;
            double rollerAngleRad = -_rollerAngle.get().toDouble()*DEG2RAD;

            double cosAngle = cos(rollerAngleRad);
            double sinAngle = sin(rollerAngleRad);

            b2Vec2 localAlongUnit = b2Vec2(cosAngle, sinAngle);
            assert(localAlongUnit.Length() - 1.0 < 0.0001);

            b2Vec2 globalAlongUnit = _wheelBody->GetWorldVector(localAlongUnit);

            //Project target velocity onto along direction
            b2Vec2 targetVelocity = _wheelBody->GetWorldVector(b2Vec2(targetLinear, 0));
            b2Vec2 targetAlongVelocity = globalAlongUnit * b2Dot(globalAlongUnit, targetVelocity);

            //Project current velocity onto along direction
            b2Vec2 alongVelocity = globalAlongUnit * b2Dot( globalAlongUnit, _wheelBody->GetLinearVelocity() );

            //Calculate impulse to move forward
            b2Vec2 impulse = (targetAlongVelocity - alongVelocity) * _wheelBody->GetMass();

            _wheelBody->ApplyLinearImpulse( impulse, _wheelBody->GetWorldCenter(), true );

            /*qDebug() << "--------------------------";
            qDebug() << "Wheel" << getName();
            qDebug() << "Applying impulse to get velocity" << targetAlongVelocity << "in direction" << localAlongUnit;
            qDebug() << "Target angular:" << targetTheta;
            qDebug() << "Target linear: " << targetLinear;
            qDebug() << "Actual linear: " << _wheelBody->GetLinearVelocity();
            qDebug() << "Target Along: " << targetAlongVelocity;
            qDebug() << "Actual Along: " << alongVelocity;
            qDebug() << "Impulse: " << impulse;*/
        }
    }
}

void Swedish_Wheel::_processMessage(const std_msgs::msg::Float32::SharedPtr data)
{
    _targetAngularVelocity = static_cast<double>(data->data);
}
