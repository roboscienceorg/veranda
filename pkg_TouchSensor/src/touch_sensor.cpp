#include "touch_sensor.h"
#include "std_msgs/ByteMultiArray.h"
#include <QDebug>

Touch_Sensor::Touch_Sensor(QObject *parent) : Sensor_If(parent)
{
    connect(&output_channel, &Property::valueSet, this, &Touch_Sensor::_channelChanged);
    connect(&radius, &Property::valueSet, this, &Touch_Sensor::_attachSensorFixture);
}

WorldObject_If* Touch_Sensor::clone(QObject *newParent)
{
    Touch_Sensor* out = new Touch_Sensor(newParent);

    out->output_channel.set(output_channel.get());
    out->angle_end.set(angle_end.get());
    out->angle_start.set(angle_start.get());
    out->sensor_count.set(sensor_count.get());
    out->radius.set(radius.get());

    return out;
}

void Touch_Sensor::_channelChanged(QVariant)
{
    if(_connected)
    {
        disconnectChannels();
        connectChannels();
    }
}

void Touch_Sensor::connectChannels()
{
    if(_connected)
        disconnectChannels();

    _sendChannel = _rosNode.advertise<std_msgs::ByteMultiArray>(_outputChannel.toStdString(), 5);
    _connected = true;
}

void Touch_Sensor::disconnectChannels()
{
    _sendChannel.shutdown();
    _connected = false;
}

QVector<b2JointDef*> Touch_Sensor::setDynamicBodies(QVector<b2Body *> & bodies)
{
    sensorBody = bodies.at(0);

    _attachSensorFixture();
    return {};
}

void Touch_Sensor::_attachSensorFixture()
{
    if(sensorBody)
    {
        if(sensorFix)
        {
            sensorBody->DestroyFixture(sensorFix);
        }

        b2FixtureDef fixDef;
        b2CircleShape sensorRing;

        sensorRing.m_radius = radius.get().toDouble();
        sensorRing.m_p = b2Vec2(0, 0);

        fixDef.isSensor = true;
        fixDef.shape = &sensorRing;

        sensorBody->CreateFixture(&fixDef);
    }
}

void Touch_Sensor::_buildButtonModel()
{

}

void Touch_Sensor::_updateTouchesModel()
{

}

void Touch_Sensor::clearDynamicBodies()
{
    sensorBody = nullptr;
    sensorFix = nullptr;
}

void Touch_Sensor::worldTicked(const b2World*, const double& t)
{
    if(sensorBody)
    {

    }
    _updateTouchesModel();
}
