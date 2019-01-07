//! \file
#pragma once

#include <veranda_core/api/model.h>
#include <Box2D/Box2D.h>

#include <QVector>

/*!
 * \brief Collection of functions used in plugins that provide wheels
 * The Basic_Wheel can be used to create shapes and models that look like
 * wheels, as well as manage the no-slip and no-slide constraints on wheels
 * in the physics engine
 */
class Basic_Wheel
{
    constexpr static double PI = 3.14159265359;
    constexpr static double RAD2DEG = 360.0/(2*PI);
    constexpr static double DEG2RAD = 1.0/RAD2DEG;

public:
    /*!
     * \brief Creates a new Model that looks like a wheel
     * The wheel consists of a rectangle with the width and twice
     * the radius of the wheel and a line from the center of the wheel
     * to the front-middle
     *
     * \param[in] radius Radius of the wheel (meters)
     * \param[in] width Width of the wheel (meters)
     * \return A newly allocated Model populated with some shapes
     */
    static Model* makeWheelModel(double radius, double width)
    {
        b2Shape* sh = makeWheelShape(radius, width);

        b2EdgeShape* line = new b2EdgeShape;
        line->m_vertex1 = b2Vec2(0, 0);
        line->m_vertex2 = b2Vec2(radius, 0);

        return new Model({}, {sh, line});
    }

    /*!
     * \brief Makes a b2Shape for the outline of a wheel
     *
     * \param[in] radius Radius of the wheel (meters)
     * \param[in] width Width of the wheel (meters)
     * \return A newly allocated b2Shape the size of the wheel
     */
    static b2Shape* makeWheelShape(double radius, double width)
    {
        b2PolygonShape* sh = new b2PolygonShape;
        sh->SetAsBox(radius, width/2.0);

        return sh;
    }

    /*!
     * \brief Applies the no-slide constraint
     * One of the properties of a wheel is that it generally doesn't
     * slide along the ground perpendicular to the direction of rolling.
     * This function takes a b2body and applies forces to prevent this
     * non-physical movement.
     *
     * It's difficult to achieve this effect perfectly, so the function
     * applies force to the body which is proportional to the mass attached
     *
     * \param[in,out] wheel The b2Body wheel to apply the constraint to
     * \param[in] radius Radius of the wheel (meters)
     */
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

    /*!
     * \brief Applies the no-slip constraint
     * One of the properties of a wheel is that it generally doesn't
     * slip; the amount that it spins translates directly to a distance traveled.
     * This function takes a b2body and applies forces to prevent this
     * non-physical movement, given a specific target rotational velocity.
     *
     * It's difficult to achieve this effect perfectly, so the function
     * applies force to the body which is proportional to the mass attached
     *
     * \param[in,out] wheel The b2Body wheel to apply the constraint to
     * \param[in] radius Radius of the wheel (meters)
     * \param[in] dtheta_rad Target rotational velocity of the wheel (rad/s)
     */
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
