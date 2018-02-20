#ifndef BASIC_WHEEL_H
#define BASIC_WHEEL_H

#include <sdsmt_simulator/model.h>
#include <Box2D/Box2D.h>

#include <QVector>

class Basic_Wheel
{
    constexpr static double PI = 3.14159265359;
    constexpr static double RAD2DEG = 360.0/(2*PI);
    constexpr static double DEG2RAD = 1.0/RAD2DEG;

public:
    static Model* makeWheelModel(double radius, double width)
    {
        b2Shape* sh = makeWheelShape(radius, width);

        b2EdgeShape* line = new b2EdgeShape;
        line->m_vertex1 = b2Vec2(0, 0);
        line->m_vertex2 = b2Vec2(radius, 0);

        return new Model({}, {sh, line});
    }

    static b2Shape* makeWheelShape(double radius, double width)
    {
        b2PolygonShape* sh = new b2PolygonShape;
        sh->SetAsBox(radius, width/2.0);

        return sh;
    }

    static void applyNoSlideConstraint(b2Body* wheel, double radius)
    {
        static b2Vec2 _localWheelRightUnit = b2Vec2(0, 1);

        b2Vec2 right = wheel->GetWorldVector(_localWheelRightUnit);

        //Calculate lateral movement
        b2Vec2 lateralVelocity = right * b2Dot( right, wheel->GetLinearVelocity() );

        //Negate slip/slide
        b2Vec2 impulse = -lateralVelocity * wheel->GetMass();
        wheel->ApplyLinearImpulse( impulse, wheel->GetWorldCenter(), true );
    }

    static void applyNoSlipConstraint(b2Body* wheel, double radius, double dtheta_rad)
    {
        static b2Vec2 _localWheelFrontUnit = b2Vec2(1, 0);

        b2Vec2 front = wheel->GetWorldVector(_localWheelFrontUnit);

        //Calculation to find target velocity from
        //rotations per second; original idea was to behave more line
        //real moter controller
        //Circumference * ratio of 2PI to radians traveled
        double targetVelocity = 2*PI*radius * dtheta_rad/(2*PI);

        //Calculate linear movement
        b2Vec2 forwardVelocity = front * b2Dot( front, wheel->GetLinearVelocity() );

        //Negate slip
        b2Vec2 impulse = (front * targetVelocity - forwardVelocity) * wheel->GetMass();
        wheel->ApplyLinearImpulse( impulse, wheel->GetWorldCenter(), true );
    }
};

#endif // BASIC_WHEEL_H
