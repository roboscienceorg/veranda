#include "gps_sensor.h"

#include <QDebug>

#include <cmath>
#include <limits>

GPS_Sensor::GPS_Sensor(QObject *parent) : WorldObjectComponent("GPS", "Sensors", parent)
{
    //Update channel out
    connect(&output_channel, &Property::valueSet, this, &GPS_Sensor::_channelChanged);

    //Reset publish timer when rate changes
    connect(&pub_rate, &Property::valueSet, [this](){ _timeSincePublish = 0; });

    sensor_model = new Model({}, {}, this);
    registerModel(sensor_model);
    _buildModels();

    data = std::make_shared<geometry_msgs::msg::Pose2D>();
}

WorldObjectComponent *GPS_Sensor::_clone(QObject *newParent)
{
    GPS_Sensor* out = new GPS_Sensor(newParent);

    return out;
}

void GPS_Sensor::setROSNode(std::shared_ptr<rclcpp::Node> node)
{
    _sendChannel.reset();
    _rosNode = node;
}

void GPS_Sensor::_channelChanged()
{
    if(_sendChannel)
    {
        connectChannels();
    }
}

void GPS_Sensor::connectChannels()
{
    disconnectChannels();

    if(_rosNode)
    {
        _outputChannel = output_channel.get().toString();

        if(_outputChannel.size())
        {
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
            custom_qos_profile.depth = 7;

            _sendChannel = _rosNode->create_publisher<geometry_msgs::msg::Pose2D>(_outputChannel.toStdString(), custom_qos_profile);
        }
    }
}

void GPS_Sensor::disconnectChannels()
{
    _sendChannel.reset();
}

void GPS_Sensor::clearBodies()
{
    if(_world)
    {
        _world->DestroyJoint(weldJoint);
        _world->DestroyBody(sensorBody);
        unregisterBody(sensorBody);
        weldJoint = nullptr;
        sensorBody = nullptr;
        sensorFix = nullptr;
    }
    _world = nullptr;
}

void GPS_Sensor::generateBodies(b2World *world, object_id oId, b2Body *anchor)
{
    clearBodies();
    _world = world;

    b2BodyDef bDef;
    bDef.type = b2_dynamicBody;
    sensorBody = world->CreateBody(&bDef);
    registerBody(sensorBody, {sensor_model}, true);

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

void GPS_Sensor::_attachSensorFixture()
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
        b2CircleShape sensorCircle;

        sensorCircle.m_radius = SHAPE_RADIUS;
        sensorCircle.m_p = b2Vec2(0, 0);

        fixDef.isSensor = true;
        fixDef.shape = &sensorCircle;
        fixDef.filter.groupIndex = -objectId;
        fixDef.density = 1;

        sensorFix = sensorBody->CreateFixture(&fixDef);
    }
}

void GPS_Sensor::_buildModels()
{
    //Clear always-showing model
    QVector<b2Shape*> oldModel = sensor_model->shapes();
    sensor_model->removeShapes(oldModel);
    qDeleteAll(oldModel);

    //Build always-showing model...
    b2CircleShape* ring = new b2CircleShape;
    ring->m_radius = SHAPE_RADIUS;
    ring->m_p = b2Vec2(0,0);

    double x = sqrt(2.0)/2.0 * SHAPE_RADIUS;

    b2EdgeShape* line = new b2EdgeShape;
    line->m_vertex1.Set(x, x);
    line->m_vertex2.Set(-x, -x);
    sensor_model->addShapes({ring, line});

    line = new b2EdgeShape;
    line->m_vertex1.Set(-x, x);
    line->m_vertex2.Set(x, -x);
    sensor_model->addShapes({line});

    line = new b2EdgeShape;
    line->m_vertex1.Set(0, SHAPE_RADIUS);
    line->m_vertex2.Set(0, -SHAPE_RADIUS);
    sensor_model->addShapes({line});
}

double GPS_Sensor::observe(const double& actual, const double& chance,
               double &drift, const double& drift_sigma, const double& drift_mu, const double &drift_scale,
               const double& noise_sigma, const double& noise_mu)
{
    double newDrift = std::normal_distribution<>(drift_mu, drift_sigma)(_reng) * drift_scale;
    drift += newDrift;

    double beNan = _uniDist(_reng);
    if(beNan >= chance) return std::numeric_limits<double>::quiet_NaN();

    return actual + drift + std::normal_distribution<>(noise_mu, noise_sigma)(_reng);
}

void GPS_Sensor::_worldTicked(const double dt)
{
    if(sensorBody)
    {
        _timeSincePublish += dt;
        double pr = pub_rate.get().toDouble();

        if(pr > 0 && _timeSincePublish > 1.0/pr)
        {
            //Observe current location, updating drift and calculating
            //values to be published
            data->x = observe(sensorBody->GetPosition().x, x_chance.get().toDouble(),
                              _drift_x, x_drift_sigma.get().toDouble(), x_drift_mu.get().toDouble(), _timeSincePublish,
                              x_noise_sigma.get().toDouble(), x_noise_mu.get().toDouble());

            data->y = observe(sensorBody->GetPosition().y, y_chance.get().toDouble(),
                              _drift_y, y_drift_sigma.get().toDouble(), y_drift_mu.get().toDouble(), _timeSincePublish,
                              y_noise_sigma.get().toDouble(), y_noise_mu.get().toDouble());

            data->theta = observe(sensorBody->GetAngle(), t_chance.get().toDouble(),
                              _drift_t, t_drift_sigma.get().toDouble(), t_drift_mu.get().toDouble(), _timeSincePublish,
                              t_noise_sigma.get().toDouble(), t_noise_mu.get().toDouble());

            if(_sendChannel)
            {
                _sendChannel->publish(data);
            }

            _timeSincePublish = 0;
        }
    }
}
