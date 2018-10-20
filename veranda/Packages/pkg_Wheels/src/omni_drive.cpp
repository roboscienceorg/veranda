#include "omni_drive.h"
#include "basic_wheel.h"

#include <QDebug>
#include <QSharedPointer>
#include <cmath>

Omni_Drive::Omni_Drive(QObject *parent) :
    WorldObjectComponent("Omni Drive", "Wheels", parent),
#define pview(a) QSharedPointer<PropertyView>(new PropertyView(a))
    drive_filter(pview(&in_noise_mu), pview(&in_noise_sigma), 1),
    report_filter(pview(&out_noise_mu), pview(&out_noise_sigma), 1)
#undef pview
{
    qRegisterMetaType<geometry_msgs::msg::Pose2D::SharedPtr>("geometry_msgs::msg::Pose2D::SharedPtr");

    //Update channel in if name or driven status changes
    connect(&_inputChannel, &Property::valueSet, this, &Omni_Drive::_refreshChannel);
    connect(&_outputChannel, &Property::valueSet, this, &Omni_Drive::_refreshChannel);
    connect(&_driven, &Property::valueSet, this, &Omni_Drive::_refreshChannel);

    //Update box2d interface
    connect(&_radius, &Property::valueSet, this, &Omni_Drive::_attachFixture);

    //Update drawn model
    connect(&_radius, &Property::valueSet, this, &Omni_Drive::_buildModels);

    connect(&_density, &Property::valueSet,
    [=](QVariant d)
    {
        if(_ballFix)
        {
            _ballFix->SetDensity(d.toDouble());
            _ballBody->ResetMassData();
        }
    });

    //Signal slot connection to correctly thread incomming messages
    connect(this, &Omni_Drive::_receiveMessage, this, &Omni_Drive::_processMessage);

    _ballModel = new Model({}, {}, this);
    registerModel(_ballModel);

    _buildModels();
}

void Omni_Drive::_generateBodies(b2World* world, object_id oId, b2Body* anchor)
{
    clearBodies();
    _world = world;

    b2BodyDef bDef;
    bDef.type = b2_dynamicBody;
    _ballBody = world->CreateBody(&bDef);
    registerBody(_ballBody, {_ballModel}, true);

    b2WeldJointDef weldDef;
    auto anchorPt = anchor->GetWorldCenter();
    weldDef.bodyA = anchor;
    weldDef.bodyB = _ballBody;
    weldDef.localAnchorA = anchor->GetLocalPoint(anchorPt);
    weldDef.localAnchorB = _ballBody->GetLocalPoint(anchorPt);
    weldDef.referenceAngle = _ballBody->GetAngle() - anchor->GetAngle();
    _weldJoint = world->CreateJoint(&weldDef);

    _objectId = oId;

    _attachFixture();
}

void Omni_Drive::_clearBodies()
{
    if(_world)
    {
        _world->DestroyJoint(_weldJoint);
        _world->DestroyBody(_ballBody);
        unregisterBody(_ballBody);
        _weldJoint = nullptr;
        _ballBody = nullptr;
        _ballFix = nullptr;
    }
    _world = nullptr;
}

void Omni_Drive::_attachFixture()
{
    //Can only add fixture if body defined
    if(_ballBody)
    {
        //If old fixture, remove it
        if(_ballFix)
        {
            _ballBody->DestroyFixture(_ballFix);
            _ballFix = nullptr;
        }

        //Create rectangle to represent the wheel from top down
        b2FixtureDef fixDef;
        b2Shape* wheel = new b2CircleShape();
        wheel->m_radius = _radius.get().toDouble();

        fixDef.isSensor = false;
        fixDef.shape = wheel;
        fixDef.filter.groupIndex = -_objectId;
        fixDef.density = _density.get().toDouble();

        _ballFix = _ballBody->CreateFixture(&fixDef);

        delete wheel;
    }
}

WorldObjectComponent *Omni_Drive::_clone(QObject *newParent)
{
    Omni_Drive* out = new Omni_Drive(newParent);

    return out;
}

void Omni_Drive::_setROSNode(std::shared_ptr<rclcpp::Node> node)
{
    _receiveChannel.reset();
    _publishChannel.reset();
    _rosNode = node;
}

void Omni_Drive::_refreshChannel(QVariant)
{
    if(_receiveChannel || _publishChannel)
    {
        connectChannels();
    }
}

void Omni_Drive::_connectChannels()
{
    //qDebug() << "Connecting Fixed Wheel Channels";
    disconnectChannels();

    if(_rosNode)
    {
        //Only listen when the wheel is driven
        //qDebug() << _driven.get().toBool() << " && " << (bool)_rosNode;
        if(_driven.get().toBool())
        {
            QString inputChannel = _inputChannel.get().toString();

            if(inputChannel.size())
            {
                auto callback =
                [this](const geometry_msgs::msg::Pose2D::SharedPtr msg) -> void
                {
                    _receiveMessage(msg);
                };

                _receiveChannel = _rosNode->create_subscription<geometry_msgs::msg::Pose2D>(inputChannel.toStdString(), callback);

                //qDebug() << "Channel: " << inputChannel << " created: " << _receiveChannel.get();
            }
        }

        QString outputChannel = _outputChannel.get().toString();

        if(outputChannel.size())
        {
            _publishChannel = _rosNode->create_publisher<geometry_msgs::msg::Pose2D>(outputChannel.toStdString(), 10);
        }
    }
    //qDebug() << "";
    _timeSincePublish = 0;
}

void Omni_Drive::_disconnectChannels()
{
    //qDebug() << "Channel: " << _inputChannel.get().toString() << " deleted";
    _receiveChannel.reset();
    _publishChannel.reset();
}

void Omni_Drive::_buildModels()
{
    _ballModel->removeShapes(_ballShapes);
    qDeleteAll(_ballShapes);
    _ballShapes.clear();

    double r = _radius.get().toDouble();
    b2CircleShape* baseCircle = new b2CircleShape();
    baseCircle->m_radius = r;

    double s = sqrt(2) * r/2;
    b2PolygonShape* inscribe = new b2PolygonShape();
    inscribe->SetAsBox(s, s);

    _ballShapes << baseCircle << inscribe;

    _ballModel->addShapes(_ballShapes);
}

void Omni_Drive::_worldTicked(const double dt)
{
    if(_ballBody)
    {
        auto linVelocity = _ballBody->GetLinearVelocity();
        auto rotVelocity = _ballBody->GetAngularVelocity();

        if(_driven.get().toBool())
        {
            double x_impulse = _targetXVelocity - linVelocity.x;
            double y_impulse = _targetYVelocity - linVelocity.y;

            b2Vec2 impulse = b2Vec2(x_impulse, y_impulse) * _ballBody->GetMass();

            _ballBody->ApplyLinearImpulse( impulse, _ballBody->GetWorldCenter(), true );

            double t_impulse = (_targetAngularVelocity - rotVelocity)*_ballBody->GetMass();
            _ballBody->ApplyAngularImpulse(t_impulse, true);
        }

        double timePerMsg = 1.0/pub_rate.get().toDouble();
        _timeSincePublish += dt;

        if(_timeSincePublish >= timePerMsg)
        {
            _timeSincePublish = 0;
            if(_publishChannel)
            {
                auto newMsg = std::make_shared<geometry_msgs::msg::Pose2D>();

                newMsg->theta = report_filter.apply() + rotVelocity;
                newMsg->x = report_filter.apply() + linVelocity.x;
                newMsg->y = report_filter.apply() + linVelocity.y;

                _publishChannel->publish(newMsg);
            }
        }
    }
}

void Omni_Drive::_processMessage(const geometry_msgs::msg::Pose2D::SharedPtr data)
{
    _targetAngularVelocity = static_cast<double>(data->theta) + drive_filter.apply();
    _targetXVelocity = static_cast<double>(data->x) + drive_filter.apply();
    _targetYVelocity = static_cast<double>(data->y) + drive_filter.apply();
}
