#include "encoder.h"


Encoder::Encoder(QObject *parent) :
    WorldObjectComponent("Encoder", "Sensors", parent),
    _sendMessage(std::make_shared<std_msgs::msg::Float32>()),
#define pview(a) QSharedPointer<PropertyView>(new PropertyView(a))
    noise_filter(pview(&noise_mu), pview(&noise_sigma), 1.0)
#undef pview
{
    qRegisterMetaType<std_msgs::msg::Float32::SharedPtr>("std_msgs::msg::Float32::SharedPtr");

    //Update channel in if name or driven status changes
    connect(&output_channel, &Property::valueSet, this, &Encoder::_refreshChannel);
}

WorldObjectComponent *Encoder::_clone(QObject *newParent)
{
    Encoder* out = new Encoder(newParent);

    return out;
}

double Encoder::_calculateAngularVelocity(const b2Body* body, const double& radius)
{
    static b2Vec2 _localWheelFrontUnit = b2Vec2(1, 0);

    b2Vec2 front = body->GetWorldVector(_localWheelFrontUnit);

    //Calculate linear movement
    double forwardVelocity = static_cast<double>((front * b2Dot( front, body->GetLinearVelocity() )).Length());

    double angularVelocity = forwardVelocity/(2*PI*radius) * (2*PI);

    return angularVelocity;
}

void Encoder::setROSNode(std::shared_ptr<rclcpp::Node> node)
{
    _sendChannel.reset();
    _rosNode = node;
}

void Encoder::_refreshChannel(QVariant)
{
    if(_sendChannel)
    {
        connectChannels();
    }
}

void Encoder::connectChannels()
{
    disconnectChannels();

    if(_rosNode)
    {
        QString outputChannel = output_channel.get().toString();

        if(outputChannel.size())
        {
            _sendChannel = _rosNode->create_publisher<std_msgs::msg::Float32>(outputChannel.toStdString(), 10);
        }
    }
}

void Encoder::disconnectChannels()
{
    _sendChannel.reset();
}


void Encoder::_worldTicked(const double dt)
{
    if(_wheelBody && _sendChannel && pub_rate.get().toDouble() > 0)
    {
        double publishDelta = 1.0/pub_rate.get().toDouble();
        if((_lastPublish += dt) > publishDelta)
        {
            _lastPublish = 0;

            _sendMessage->data = static_cast<float>(_calculateAngularVelocity(_wheelBody, _wheelRadius));

            _sendChannel->publish(_sendMessage);
        }
    }
}

void Encoder::setWheel(b2Body* wheelBody, double radius)
{
    _wheelBody = wheelBody;
    _wheelRadius = radius;
}
