#include "touch_sensor.h"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

#include <QDebug>
#include <cmath>

Touch_Sensor::Touch_Sensor(QObject *parent) : WorldObjectComponent_If(parent)
{
    //Update channel out
    connect(&output_channel, &Property::valueSet, this, &Touch_Sensor::_channelChanged);

    //Update box2d interface
    connect(&radius, &Property::valueSet, this, &Touch_Sensor::_attachSensorFixture);

    //Update output message dimensions
    connect(&sensor_count, &Property::valueSet, this, &Touch_Sensor::_updateDataMessageDimensions);

    //Update drawn model
    connect(&radius, &Property::valueSet, this, &Touch_Sensor::_buildModels);
    connect(&sensor_count, &Property::valueSet, this, &Touch_Sensor::_buildModels);
    connect(&angle_start, &Property::valueSet, this, &Touch_Sensor::_buildModels);
    connect(&angle_end, &Property::valueSet, this, &Touch_Sensor::_buildModels);

    buttons_model = new Model({}, {}, this);
    touches_model = new Model({}, {}, this);

    data = std::make_shared<std_msgs::msg::ByteMultiArray>();

    data->layout.data_offset = 0;
    data->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    data->layout.dim[0].label = "hits";

    _updateDataMessageDimensions();
}

void Touch_Sensor::_updateDataMessageDimensions()
{
    int buttons = sensor_count.get().toInt();
    data->layout.dim[0].stride = data->layout.dim[0].size = buttons;
    data->data.clear();
    data->data.resize(buttons, 0);
}

WorldObjectComponent_If *Touch_Sensor::clone(QObject *newParent)
{
    Touch_Sensor* out = new Touch_Sensor(newParent);

    for(QString s : _properties.keys())
    {
        out->_properties[s].set(_properties[s].get(), true);
    }

    return out;
}

void Touch_Sensor::setROSNode(std::shared_ptr<rclcpp::Node> node)
{
    _sendChannel.reset();
    _rosNode = node;
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

    if(_rosNode)
    {
        _outputChannel = output_channel.get().toString();

        if(_outputChannel.size())
        {
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
            custom_qos_profile.depth = 7;

            _sendChannel = _rosNode->create_publisher<std_msgs::msg::ByteMultiArray>(_outputChannel.toStdString(), custom_qos_profile);
        }
    }
    _connected = true;

}

void Touch_Sensor::disconnectChannels()
{
    _sendChannel.reset();
    _connected = false;
}

void Touch_Sensor::clearBodies(b2World *world)
{
    if(nullptr != sensorBody)
    {
        world->DestroyJoint(weldJoint);
        world->DestroyBody(sensorBody);

        weldJoint = nullptr;
        sensorBody = nullptr;
        sensorFix = nullptr;

        massChanged(this, 0);
    }
}

void Touch_Sensor::generateBodies(b2World *world, object_id oId, b2Body *anchor)
{
    clearBodies(world);

    b2BodyDef bDef;
    bDef.angle = theta_local.get().toDouble()*DEG2RAD;
    bDef.type = b2_dynamicBody;
    sensorBody = world->CreateBody(&bDef);

    moveBodyToLocalSpaceOfOtherBody(sensorBody, anchor, x_local.get().toDouble(), y_local.get().toDouble());

    b2WeldJointDef weldDef;
    auto anchorPt = anchor->GetWorldCenter();
    weldDef.bodyA = anchor;
    weldDef.bodyB = sensorBody;
    weldDef.localAnchorA = anchor->GetLocalPoint(anchorPt);
    weldDef.localAnchorB = sensorBody->GetLocalPoint(anchorPt);
    weldDef.referenceAngle = sensorBody->GetAngle() - anchor->GetAngle();
    weldJoint = world->CreateJoint(&weldDef);

    objectId = oId;

    _attachSensorFixture();
    _buildModels();
}

void Touch_Sensor::_attachSensorFixture()
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
        fixDef.filter.groupIndex = -objectId;
        fixDef.density = 1;

        sensorFix = sensorBody->CreateFixture(&fixDef);

        massChanged(this, sensorBody->GetMass());
    }
}

void Touch_Sensor::_buildModels()
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

    //zero out all output data
    active_touches.clear();
    data->data.clear();
    data->data.resize(buttons, 0);
}

void Touch_Sensor::_evaluateContact(b2Contact* c, QVector<int>& newTouches, QSet<int>& touchesNow)
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
            if(!data->data[sensorHit])
            {
                data->data[sensorHit] = 1;
                newTouches.push_back(sensorHit);
            }
        }
    }
}

void Touch_Sensor::worldTicked(const b2World*, const double)
{
    if(sensorBody)
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
            data->data[i] = 0;
            oldHitModels.push_back(touch_image[i]);
        }

        if(newHitModels.size() || oldHitModels.size())
        {
            touches_model->addShapes(newHitModels);
            touches_model->removeShapes(oldHitModels);
        }
        active_touches = currentTouches;

        if(anyChange && _sendChannel)
        {
            _sendChannel->publish(data);
        }

        double x = sensorBody->GetWorldCenter().x;
        double y = sensorBody->GetWorldCenter().y;
        double t = sensorBody->GetAngle();
        buttons_model->setTransform(x, y, t*RAD2DEG);
        touches_model->setTransform(x, y, t*RAD2DEG);
    }
}
