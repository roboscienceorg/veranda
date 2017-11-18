#include "robot.h"

#include <QDebug>

Robot::Robot(QVector<b2Shape *> body, DriveTrain_If* dt, QVector<Sensor_If*> sensors, QObject* parent) :
    depracatedWorldObject_If(parent), _mainShapes(body), _drivetrain(dt), _sensors(sensors)
{
    _model = new Model({}, body, this);
    _allModels.push_back(_model);

    connect(_drivetrain, &DriveTrain_If::targetVelocity, this, &Robot::targetVelocity);
    _bodiesRequired += dt->dynamicBodiesRequired();
    _drivetrain->setParent(this);
    connect(_drivetrain, &DriveTrain_If::massChanged, this, &Robot::computeMass);

    QMap<QString, PropertyView> props = dt->getAllProperties();
    for(auto iter = props.begin(); iter != props.end(); iter++)
    {
        _properties.insert(dt->propertyGroupName() + "/" + iter.key(), iter.value());
    }
    _allModels += _drivetrain->getModels();

    for(Sensor_If* s : _sensors)
    {
        _bodiesRequired += s->dynamicBodiesRequired();
        s->setParent(this);
        connect(s, &Sensor_If::massChanged, this, &Robot::computeMass);

        props = s->getAllProperties();
        for(auto iter = props.begin(); iter != props.end(); iter++)
            _properties.insert(s->propertyGroupName() + "/" + iter.key(), iter.value());
        _allModels += s->getModels();
    }
}

Robot::~Robot()
{
    qDeleteAll(_mainShapes);
}

void Robot::setOrientation(double x0, double y0, double theta0)
{
    _x0 = x0;
    _y0 = y0;
    _theta0 = theta0 * DEG2RAD;
}

depracatedWorldObject_If* Robot::clone(QObject *newParent)
{
    DriveTrain_If* newDt = static_cast<DriveTrain_If*>(_drivetrain->clone());

    QVector<Sensor_If*> newSensors;
    for(Sensor_If* s : _sensors)
        newSensors.push_back(static_cast<Sensor_If*>(s->clone()));

    QVector<b2Shape*> newBody;
    for(b2Shape* s : _model->shapes())
        newBody.push_back(cloneShape(s));

    Robot* out = new Robot(newBody, newDt, newSensors, newParent);
    out->setOrientation(_x0, _y0, _theta0*RAD2DEG);
    return out;
}

void Robot::clearDynamicBodies()
{
    _mainBody = nullptr;
    _allBodies.clear();
    _drivetrain->clearDynamicBodies();
    for(Sensor_If* s : _sensors)
        s->clearDynamicBodies();
}

b2JointDef* Robot::_defineJoint(b2Body* a, b2Body* b)
{
    b2WeldJointDef* out = new b2WeldJointDef;
    out->Initialize(a, b, a->GetPosition());
    out->collideConnected = false;
    out->frequencyHz = 0;
    return out;
}

void Robot::computeMass()
{
    if(_mainBody)
    {
        _mainBody->ResetMassData();
        totalMass = _mainBody->GetMass();
        for(b2Body* b : _allBodies)
        {
            b->ResetMassData();
            totalMass += b->GetMass();
        }

        qDebug() << "Robot mass: " << totalMass;
    }
}

QVector<b2JointDef *> Robot::setDynamicBodies(QVector<b2Body*>& bodies)
{
    _allBodies = bodies;
    QVector<b2JointDef*> joints;

    QVector<b2Body*> tmp;
    int i=0;
    for(int j=0; j< _drivetrain->dynamicBodiesRequired(); j++, i++)
    {
        bodies.at(i)->SetTransform(b2Vec2(_x0, _y0), _theta0);
        tmp.push_back(bodies.at(i));
    }
    joints += _drivetrain->setDynamicBodies(tmp);

    for(Sensor_If* s : _sensors)
    {
        tmp.clear();
        for(int j=0; j<s->dynamicBodiesRequired(); j++, i++)
        {
            bodies.at(i)->SetTransform(b2Vec2(_x0, _y0), _theta0);
            tmp.push_back(bodies.at(i));
        }
        joints += s->setDynamicBodies(tmp);
    }

    _mainBody = bodies.at(i);
    for(b2Shape* s : _model->shapes())
    {
        b2FixtureDef bodyDef;
        bodyDef.shape = s;
        bodyDef.density = 1.0;
        bodyDef.friction = 0.0;

        _mainBody->CreateFixture(&bodyDef);
    }

    computeMass();

    _mainBody->SetTransform(b2Vec2(_x0, _y0), _theta0);

    //Weld all sensor and drivetrain bodies to main body
    for(int j = 0; j < i; j++)
    {
        joints.push_back(_defineJoint(_mainBody, bodies[j]));
    }

    qDebug() << "Starting robot at " << _x0 << _y0 << _theta0;

    return joints;
}

void Robot::connectChannels()
{
    _drivetrain->connectChannels();
    for(Sensor_If* s : _sensors)
        s->connectChannels();
}

void Robot::disconnectChannels()
{
    _drivetrain->disconnectChannels();
    for(Sensor_If* s : _sensors)
        s->disconnectChannels();
}

void Robot::targetVelocity(double x, double y, double theta)
{
    if(_mainBody && _lastTick != 0)
    {
        //Translate to world coordinate velocities
        if(!_drivetrain->usesWorldCoords())
        {
            double sint = sin(_theta);
            double cost = cos(_theta);

            //qDebug() << "Robot velocity to world velocity" << _theta << sint << cost;
            //qDebug() << x << y;

            double x_ = cost*x + -sint*y;
            y = sint*x + cost*y;
            x = x_;

            //qDebug() << x << y;
        }
        if(_drivetrain->usesDegrees())
        {
            theta *= DEG2RAD;
        }

        b2Vec2 currLinVelocity = _mainBody->GetLinearVelocity();
        b2Vec2 impulseLin((x - currLinVelocity.x), (y - currLinVelocity.y));
        _mainBody->ApplyLinearImpulseToCenter(impulseLin, true);

        double currAngVelocity = _mainBody->GetAngularVelocity();
        double impulseAng = _mainBody->GetInertia() * (theta - currAngVelocity);
        _mainBody->ApplyAngularImpulse(impulseAng, true);

        /*qDebug() << "Target velocity:" << x << y << theta;
        qDebug() << "Current velocity:" << currLinVelocity.x << currLinVelocity.y << currAngVelocity;
        qDebug() << "Diff velocity:" << (x - currLinVelocity.x) << (y - currLinVelocity.y) << (theta - currAngVelocity);
        qDebug() << "Apply impulses:" << impulseLin.x << impulseLin.y << impulseAng;*/
    }
}

void Robot::worldTicked(const b2World* world, const double& t)
{
    if(_mainBody)
    {
        _drivetrain->worldTicked(world, t);
        for(Sensor_If* s : _sensors) s->worldTicked(world, t);

        _x = _mainBody->GetWorldCenter().x;
        _y = _mainBody->GetWorldCenter().y;
        _theta = _mainBody->GetAngle();

        while(_theta < 0) _theta += 2*PI;
        while(_theta > 2*PI) _theta -= 2*PI;

        //qDebug() << _x << _y << _theta;

        _model->setTransform(_x, _y, _theta*RAD2DEG);

        _lastTick = t;
    }
}
