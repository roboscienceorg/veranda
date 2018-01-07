#include "fixed_wheel.h"
#include "std_msgs/ByteMultiArray.h"

#include <QDebug>
#include <cmath>

Fixed_Wheel::Fixed_Wheel(QObject *parent) : WorldObjectComponent_If(parent)
{
    //Update channel out
    connect(&input_channel, &Property::valueSet, this, &Fixed_Wheel::_channelChanged);

    //Update box2d interface
    connect(&radius, &Property::valueSet, this, &Fixed_Wheel::_attachWheelFixture);
    connect(&width, &Property::valueSet, this, &Fixed_Wheel::_attachWheelFixture);

    //Update drawn model
    connect(&radius, &Property::valueSet, this, &Fixed_Wheel::_buildModels);
    connect(&width, &Property::valueSet, this, &Fixed_Wheel::_buildModels);



    buttons_model = new Model({}, {}, this);
    touches_model = new Model({}, {}, this);

    data.layout.data_offset = 0;
    data.layout.dim.push_back(std_msgs::MultiArrayDimension());
    data.layout.dim[0].label = "hits";

    _updateDataMessageDimensions();
}

void Fixed_Wheel::_updateDataMessageDimensions()
{
    int buttons = sensor_count.get().toInt();
    data.layout.dim[0].stride = data.layout.dim[0].size = buttons;
    data.data.clear();
    data.data.resize(buttons, 0);
}

WorldObjectComponent_If *Fixed_Wheel::clone(QObject *newParent)
{
    Fixed_Wheel* out = new Fixed_Wheel(newParent);

    out->output_channel.set(output_channel.get());
    out->angle_end.set(angle_end.get());
    out->angle_start.set(angle_start.get());
    out->sensor_count.set(sensor_count.get());
    out->radius.set(radius.get());

    return out;
}

void Fixed_Wheel::_channelChanged(QVariant)
{
    if(_connected)
    {
        disconnectChannels();
        connectChannels();
    }
}

void Fixed_Wheel::connectChannels()
{
    if(_connected)
        disconnectChannels();

    _outputChannel = output_channel.get().toString();
    _sendChannel = _rosNode.advertise<std_msgs::ByteMultiArray>(_outputChannel.toStdString(), 5);
    _connected = true;
}

void Fixed_Wheel::disconnectChannels()
{
    _sendChannel.shutdown();
    _connected = false;
}

QVector<b2JointDef*> Fixed_Wheel::setDynamicBodies(QVector<b2Body *> & bodies)
{
    sensorBody = bodies.at(0);

    _attachSensorFixture();
    _buildModels();
    return {};
}

void Fixed_Wheel::_attachSensorFixture()
{
    //Can only add fixture if body defined
    if(sensorBody)
    {
        //If old fixture, remove it
        if(sensorFix)
        {
            sensorBody->DestroyFixture(sensorFix);
            sensorFix = nullptr;
        }

        //Create circle at ring radius
        b2FixtureDef fixDef;
        b2CircleShape sensorRing;

        sensorRing.m_radius = radius.get().toDouble();
        sensorRing.m_p = b2Vec2(0, 0);

        fixDef.isSensor = false;
        fixDef.shape = &sensorRing;
        fixDef.density = 0.0001;

        sensorFix = sensorBody->CreateFixture(&fixDef);
    }
}

void Fixed_Wheel::_buildModels()
{
    if(sensorBody)
    {
        //Clear always-showing model
        QVector<b2Shape*> oldModel = buttons_model->shapes();
        buttons_model->removeShapes(oldModel);
        qDeleteAll(oldModel);

        //Build always-showing model...
        b2CircleShape* ring = new b2CircleShape;
        ring->m_radius = radius.get().toDouble();
        ring->m_p = b2Vec2(0,0);
        buttons_model->addShapes({ring});

        //Clear sensor-hits shapes
        touches_model->removeShapes(touches_model->shapes());
        qDeleteAll(touch_image);

        //Build new sensor-hits shapes
        touch_image.clear();

        int buttons = sensor_count.get().toInt();
        double angleMin = angle_start.get().toDouble() * DEG2RAD;
        double angleMax = angle_end.get().toDouble() * DEG2RAD;
        double degPerTouch = (angleMax - angleMin) / buttons;
        if(angleMax < angleMin)
            degPerTouch = (angleMax + 2*PI - angleMin) / buttons;

        double currAngle = angleMin + degPerTouch/2.0;
        double touchRadius = radius.get().toDouble();
        for(int i=0; i<buttons; i++, currAngle += degPerTouch)
        {
            b2CircleShape* hit = new b2CircleShape;
            hit->m_radius = 0.25;
            hit->m_p.x = touchRadius * std::cos(currAngle);
            hit->m_p.y = touchRadius * std::sin(currAngle);

            touch_image.push_back(hit);
        }

        double x = sensorBody->GetWorldCenter().x;
        double y = sensorBody->GetWorldCenter().y;
        double t = sensorBody->GetAngle();
        buttons_model->setTransform(x, y, t*RAD2DEG);
        touches_model->setTransform(x, y, t*RAD2DEG);

        //zero out all output data
        active_touches.clear();
        data.data.clear();
        data.data.resize(buttons, 0);
    }
}

QVariant Fixed_Wheel::_validate_angle(QVariant _old, QVariant _new)
{
    bool isDouble;
    double asDouble = _new.toDouble(&isDouble);
    if(isDouble && asDouble >= 0 && asDouble <= 360)
        return _new;
    return _old;
}

void Fixed_Wheel::_evaluateContact(b2Contact* c, QVector<int>& newTouches, QSet<int>& touchesNow)
{
    b2WorldManifold man;
    c->GetWorldManifold(&man);

    b2Vec2 localCenter = sensorBody->GetLocalCenter();
    double angleMin = angle_start.get().toDouble() * DEG2RAD;
    double angleMax = angle_end.get().toDouble() * DEG2RAD;
    double degPerTouch = (angleMax - angleMin) / sensor_count.get().toInt();
    if(angleMax < angleMin)
        degPerTouch = (angleMax + 2*PI - angleMin) / sensor_count.get().toInt();

    //qDebug() << "Contact with" << c->GetManifold()->pointCount << "points";
    for(int i = 0; i < c->GetManifold()->pointCount; i++)
    {
        b2Vec2 worldContact = man.points[i];
        b2Vec2 localContact = sensorBody->GetLocalPoint(worldContact);

        //qDebug() << "Touch at " << worldContact.x << worldContact.y << "->" << localContact.x << localContact.y;

        double angle = atan2(localContact.y - localCenter.y, localContact.x - localCenter.x);
        if(angle < 0)
            angle = PI + (PI + angle);

        //qDebug() << "Angle:" << (angle * RAD2DEG);

        if((angleMin < angleMax && angle >= angleMin && angle <= angleMax) ||
           (angleMin > angleMax && (angle >= angleMin || angle <= angleMax)))
        {
            int sensorHit = (angle - angleMin) / degPerTouch;
            if(angleMin > angleMax && angle <= angleMax)
                sensorHit = (angle + 2*PI - angleMin) / degPerTouch;

            touchesNow.insert(sensorHit);
            if(!data.data[sensorHit])
            {
                data.data[sensorHit] = 1;
                newTouches.push_back(sensorHit);
            }
        }
    }
}

void Fixed_Wheel::worldTicked(const b2World*, const double&)
{
    if(sensorBody)
    {
        if(_connected)
        {
            bool anyChange = false;
            QSet<int> currentTouches;
            QVector<int> newTouches;

            b2ContactEdge* edge = sensorBody->GetContactList();
            while(edge)
            {
                _evaluateContact(edge->contact, newTouches, currentTouches);
                edge = edge->next;
            }

            active_touches -= currentTouches;
            anyChange = active_touches.size() || newTouches.size();

            QVector<b2Shape*> newHitModels, oldHitModels;
            for(int i : newTouches)
            {
                newHitModels.push_back(touch_image[i]);
            }

            for(int i : active_touches)
            {
                data.data[i] = 0;
                oldHitModels.push_back(touch_image[i]);
            }

            if(newHitModels.size() || oldHitModels.size())
            {
                touches_model->addShapes(newHitModels);
                touches_model->removeShapes(oldHitModels);
            }
            active_touches = currentTouches;

            if(anyChange)
            {
                _sendChannel.publish(data);
            }
        }

        double x = sensorBody->GetWorldCenter().x;
        double y = sensorBody->GetWorldCenter().y;
        double t = sensorBody->GetAngle();
        buttons_model->setTransform(x, y, t*RAD2DEG);
        touches_model->setTransform(x, y, t*RAD2DEG);
    }
}
