#include "Catch/catch.hpp"
#include "Box2D/Box2D.h"
#include "../include/sdsmt_simulator/property.h"

#include <QDebug>
#include <QObject>

#include <random>
#include <unordered_set>

std::mt19937 reng;

TEST_CASE("Default Validators")
{
    SECTION("Double")
    {
        SECTION("From Number")
        {
            auto result = Property::double_validator(0.0, 10.0).toDouble();
            REQUIRE(result == Approx(10));

            result = Property::double_validator(0.0, -3.3).toDouble();
            REQUIRE(result == Approx(-3.3));

            result = Property::double_validator(1.0, 0.0).toDouble();
            REQUIRE(result == Approx(0));
        }

        SECTION("From String")
        {
            SECTION("Valid")
            {
                auto result = Property::double_validator(0.0, "10.0").toDouble();
                REQUIRE(result == Approx(10));

                result = Property::double_validator(0.0, "-3.3").toDouble();
                REQUIRE(result == Approx(-3.3));

                result = Property::double_validator(1.0, "0.0").toDouble();
                REQUIRE(result == Approx(0));
            }

            SECTION("Invalid")
            {
                auto result = Property::double_validator(0.0, "yes").toDouble();
                REQUIRE(result == Approx(0));

                result = Property::double_validator(0.0, "no").toDouble();
                REQUIRE(result == Approx(0));

                result = Property::double_validator(1.0, "maybe so").toDouble();
                REQUIRE(result == Approx(1.0));
            }
        }
    }

    SECTION("Integer")
    {

        SECTION("From Number")
        {
            auto result = Property::int_validator(0, 10.0).toInt();
            REQUIRE(result == 10);

            result = Property::int_validator(0, -3.3).toInt();
            REQUIRE(result == -3);

            result = Property::int_validator(1, 0.0).toInt();
            REQUIRE(result == 0);
        }

        SECTION("From String")
        {
            SECTION("Valid")
            {
                auto result = Property::int_validator(0, "10").toInt();
                REQUIRE(result == 10);

                result = Property::int_validator(0, "-3").toInt();
                REQUIRE(result == -3);

                result = Property::int_validator(1, "0").toInt();
                REQUIRE(result == 0);
            }

            SECTION("Invalid")
            {
                auto result = Property::int_validator(0, "yes").toInt();
                REQUIRE(result == 0);

                result = Property::int_validator(0, "no").toInt();
                REQUIRE(result == 0);

                result = Property::int_validator(1, "maybe so").toInt();
                REQUIRE(result == 1);
            }
        }
    }

    SECTION("Bool")
    {
        SECTION("True")
        {
            bool val = Property::bool_validator(false, true).toBool();
            REQUIRE(val == true);

            val = Property::bool_validator(false, "true").toBool();
            REQUIRE(val == true);

            val = Property::bool_validator(false, "TruE").toBool();
            REQUIRE(val == true);

            val = Property::bool_validator(false, "1").toBool();
            REQUIRE(val == true);

            val = Property::bool_validator(false, "yes").toBool();
            REQUIRE(val == true);

            val = Property::bool_validator(false, "YeS").toBool();
            REQUIRE(val == true);

            val = Property::bool_validator(false, "y").toBool();
            REQUIRE(val == true);

            val = Property::bool_validator(false, "Y").toBool();
            REQUIRE(val == true);

            val = Property::bool_validator(false, 1).toBool();
            REQUIRE(val == true);

            val = Property::bool_validator(false, 3).toBool();
            REQUIRE(val == true);

            val = Property::bool_validator(false, -5).toBool();
            REQUIRE(val == true);
        }

        SECTION("False")
        {
            bool val = Property::bool_validator(true, false).toBool();
            REQUIRE(val == false);

            val = Property::bool_validator(true, "false").toBool();
            REQUIRE(val == false);

            val = Property::bool_validator(true, "fAlsE").toBool();
            REQUIRE(val == false);

            val = Property::bool_validator(true, "0").toBool();
            REQUIRE(val == false);

            val = Property::bool_validator(true, "no").toBool();
            REQUIRE(val == false);

            val = Property::bool_validator(true, "nO").toBool();
            REQUIRE(val == false);

            val = Property::bool_validator(true, "n").toBool();
            REQUIRE(val == false);

            val = Property::bool_validator(true, "N").toBool();
            REQUIRE(val == false);

            val = Property::bool_validator(true, 0).toBool();
            REQUIRE(val == false);

            val = Property::bool_validator(true, 0.0).toBool();
            REQUIRE(val == false);
        }
    }

    SECTION("Absolute Double")
    {

        SECTION("From Number")
        {
            auto result = Property::abs_double_validator(0.0, 10.0).toDouble();
            REQUIRE(result == Approx(10));

            result = Property::abs_double_validator(0.0, -3.3).toDouble();
            REQUIRE(result == Approx(3.3));

            result = Property::abs_double_validator(1.0, 0.0).toDouble();
            REQUIRE(result == Approx(0));
        }

        SECTION("From String")
        {
            SECTION("Valid")
            {
                auto result = Property::abs_double_validator(0.0, "10.0").toDouble();
                REQUIRE(result == Approx(10));

                result = Property::abs_double_validator(0.0, "-3.3").toDouble();
                REQUIRE(result == Approx(3.3));

                result = Property::abs_double_validator(1.0, "0.0").toDouble();
                REQUIRE(result == Approx(0));
            }

            SECTION("Invalid")
            {
                auto result = Property::abs_double_validator(0.0, "yes").toDouble();
                REQUIRE(result == Approx(0));

                result = Property::abs_double_validator(0.0, "no").toDouble();
                REQUIRE(result == Approx(0));

                result = Property::abs_double_validator(1.0, "maybe so").toDouble();
                REQUIRE(result == Approx(1.0));
            }
        }
    }

    SECTION("Angle")
    {
        SECTION("Valid Angles")
        {
            double angle = Property::angle_validator(1.0, 10.0).toDouble();
            REQUIRE(angle == Approx(10));

            angle = Property::angle_validator(1.0, 40.3).toDouble();
            REQUIRE(angle == Approx(40.3));

            angle = Property::angle_validator(1.0, 27.0283).toDouble();
            REQUIRE(angle == Approx(27.0283));

            angle = Property::angle_validator(1.0, 250.49).toDouble();
            REQUIRE(angle == Approx(250.49));

            angle = Property::angle_validator(1.0, 360).toDouble();
            REQUIRE(angle == Approx(360));

            angle = Property::angle_validator(1.0, 0).toDouble();
            REQUIRE(angle == Approx(0));
        }

        SECTION("Invalid Angles")
        {
            SECTION("Negative")
            {
                double angle = Property::angle_validator(1.0, -10.0).toDouble();
                REQUIRE(angle == Approx(1));

                angle = Property::angle_validator(1.0, -40.3).toDouble();
                REQUIRE(angle == Approx(1));

                angle = Property::angle_validator(1.0, -27.0283).toDouble();
                REQUIRE(angle == Approx(1));

                angle = Property::angle_validator(1.0, -250.49).toDouble();
                REQUIRE(angle == Approx(1));
            }

            SECTION("Positive")
            {
                double angle = Property::angle_validator(1.0, 360+10.0).toDouble();
                REQUIRE(angle == Approx(1));

                angle = Property::angle_validator(1.0, 360+40.3).toDouble();
                REQUIRE(angle == Approx(1));

                angle = Property::angle_validator(1.0, 360+27.0283).toDouble();
                REQUIRE(angle == Approx(1));

                angle = Property::angle_validator(1.0, 360+250.49).toDouble();
                REQUIRE(angle == Approx(1));
            }
        }
    }
}

TEST_CASE("Value is Settable and Validator is Used")
{
    Property p(PropertyInfo(), 0.0, &Property::abs_double_validator);

    p.set(-0.3);
    REQUIRE(p.get().toDouble() == Approx(0.3));

    p.set(-25.0);
    REQUIRE(p.get().toDouble() == Approx(25.0));

    p.set(8);
    REQUIRE(p.get().toDouble() == Approx(8));

    p.set(0);
    REQUIRE(p.get().toDouble() == Approx(0));
}

TEST_CASE("Value sets")
{
    Property p;
    PropertyView v1(&p), v2(&p), v3(&p);

    QString text;
    int internalCount = 0;
    int externalCount = 0;

    QObject::connect(&p, &Property::valueSet, [&](QVariant v){REQUIRE(v.toString() == text); internalCount++;});
    QObject::connect(&v1, &PropertyView::valueSet, [&](QVariant v){REQUIRE(v.toString() == text); externalCount++;});
    QObject::connect(&v2, &PropertyView::valueSet, [&](QVariant v){REQUIRE(v.toString() == text); externalCount++;});
    QObject::connect(&v3, &PropertyView::valueSet, [&](QVariant v){REQUIRE(v.toString() == text); externalCount++;});

    SECTION("Signal from internal requests")
    {
        SECTION("In Property")
        {
            p.set(text = "newText1");
            REQUIRE(internalCount == 1);
        }

        SECTION("In multiple view")
        {
            p.set(text = "newText2");
            REQUIRE(externalCount == 3);
        }
    }

    SECTION("Signal from external requests")
    {
        SECTION("In Property")
        {
            v1.requestValue(text = "request1");
            REQUIRE(internalCount == 1);

            v2.requestValue(text = "request2");
            REQUIRE(internalCount == 2);

            v3.requestValue(text = "request3");
            REQUIRE(internalCount == 3);
        }

        SECTION("In multiple view")
        {
            v1.requestValue(text = "request1");
            REQUIRE(externalCount == 3);

            v2.requestValue(text = "request2");
            REQUIRE(externalCount == 6);

            v3.requestValue(text = "request3");
            REQUIRE(externalCount == 9);
        }
    }
}

TEST_CASE("Value requests")
{
    Property p;
    PropertyView v1(&p), v2(&p), v3(&p);

    QString text;
    int internalCount = 0;

    QObject::connect(&p, &Property::valueRequested, [&](QVariant v){REQUIRE(v.toString() == text); internalCount++;});

    SECTION("From internal requests")
    {
        p.set(text = "text1", true);
        REQUIRE(internalCount == 1);
    }

    SECTION("Not from internal requests")
    {
        p.set(text = "text1", false);
        REQUIRE(internalCount == 0);
    }

    SECTION("From mutiple views")
    {
        v1.requestValue(text = "request1");
        REQUIRE(internalCount == 1);

        v2.requestValue(text = "request2");
        REQUIRE(internalCount == 2);

        v3.requestValue(text = "request3");
        REQUIRE(internalCount == 3);
    }
}

TEST_CASE("View copy construct")
{
    Property p;
    PropertyView v1(&p), v2 = v1, v3 = v2;

    QString text;
    int externalCount = 0;

    QObject::connect(&v1, &PropertyView::valueSet, [&](QVariant v){REQUIRE(v.toString() == text); externalCount++;});
    QObject::connect(&v2, &PropertyView::valueSet, [&](QVariant v){REQUIRE(v.toString() == text); externalCount++;});
    QObject::connect(&v3, &PropertyView::valueSet, [&](QVariant v){REQUIRE(v.toString() == text); externalCount++;});

    v1.requestValue(text = "request1");
    REQUIRE(externalCount == 3);

    v2.requestValue(text = "request2");
    REQUIRE(externalCount == 6);

    v3.requestValue(text = "request3");
    REQUIRE(externalCount == 9);

    p.set(text = "request4");
    REQUIRE(externalCount == 12);
}

TEST_CASE("View auto-invalidate")
{
    Property* p = new Property();
    PropertyView v1(p), v2 = v1, v3 = v2;

    QString text;
    int externalCount = 0;

    QObject::connect(&v1, &PropertyView::valueSet, [&](QVariant v){REQUIRE(v.toString() == text); externalCount++;});
    QObject::connect(&v2, &PropertyView::valueSet, [&](QVariant v){REQUIRE(v.toString() == text); externalCount++;});
    QObject::connect(&v3, &PropertyView::valueSet, [&](QVariant v){REQUIRE(v.toString() == text); externalCount++;});

    delete p;

    v1.requestValue(text = "not happening");
    REQUIRE(externalCount == 0);

    PropertyView invalidCopy = v1;
    invalidCopy.requestValue(text = "also not happening");

    REQUIRE(v2.get().isNull());
    REQUIRE(v3.get().isNull());
}
