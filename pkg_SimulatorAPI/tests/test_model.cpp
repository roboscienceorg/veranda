#include "Catch/catch.hpp"
#include "include/sdsmt_simulator/model.h"
#include "Box2D/Box2D.h"

#include <QDebug>
#include <QObject>

#include <random>
#include <unordered_set>

std::mt19937 reng;

TEST_CASE("Get/Set Model Transform")
{
    Model testModel;

    std::uniform_real_distribution<double> locDist(-1000, 1000);
    std::uniform_real_distribution<double> degDist(0, 360);

    SECTION("Using setter correlates to result of getter")
    {
        for(int i=0; i<1000; i++)
        {
            double x = locDist(reng), y = locDist(reng);
            double t = degDist(reng);
            double xd, yd, td;

            testModel.setTransform(x, y, t);
            testModel.getTransform(xd, yd, td);

            REQUIRE(xd == Approx(x));
            REQUIRE(yd == Approx(y));
            REQUIRE(td == Approx(t));

            xd = yd = td = 0;
            testModel.setTransform(x, y, t);
            testModel.getTransform(xd, yd, td);

            REQUIRE(xd == Approx(x));
            REQUIRE(yd == Approx(y));
            REQUIRE(td == Approx(t));
        }
    }

    SECTION("Setting transform emits transformChanged()")
    {
        double dx, dy, dt;
        double count = 0;

        QObject::connect(&testModel, &Model::transformChanged,
        [&](Model* m, double _dx, double _dy, double _dt){
           REQUIRE(m == &testModel);
           dx = _dx; dy = _dy; dt = _dt;
           count++;
        });

        for(int i=0; i<1000; i++)
        {
            double xd, yd, td;
            testModel.getTransform(xd, yd, td);

            double x = locDist(reng), y = locDist(reng);
            double t = degDist(reng);
            testModel.setTransform(x, y, t);

            REQUIRE(count == i*2);
            REQUIRE(dx == Approx(std::abs(xd - x)));
            REQUIRE(dy == Approx(std::abs(yd - y)));
            REQUIRE(dt == Approx(std::abs(td - t)));

            xd = yd = td = 0;

            testModel.getTransform(xd, yd, td);
            testModel.setTransform(x, y, t);

            REQUIRE(count == i*2+1);
            REQUIRE(dx == Approx(0));
            REQUIRE(dy == Approx(0));
            REQUIRE(dt == Approx(0));
        }
    }

    SECTION("Add/Remove shapes modifies result of Model::shapes()")
    {
        std::unordered_set<b2Shape*> shapes;
        for(int i=0; i<1000; i++)
        {
            b2Shape* newShape1 = new b2EdgeShape();
            b2Shape* newShape2 = new b2EdgeShape();
            b2Shape* newShape3 = new b2EdgeShape();

            shapes.insert(newShape1);
            testModel.addShapes({newShape1});

            REQUIRE(testModel.shapes().size() == shapes.size());
            for(b2Shape* s : testModel.shapes()) REQUIRE(shapes.count(s));

            shapes.insert(newShape2);
            shapes.insert(newShape3);
            testModel.addShapes({newShape2, newShape3});

            REQUIRE(testModel.shapes().size() == shapes.size());
            for(b2Shape* s : testModel.shapes()) REQUIRE(shapes.count(s));
        }

        for(int i=0; i<1000; i++)
        {
            auto iter = shapes.begin();
            b2Shape* oldShape1 = *(iter++);
            b2Shape* oldShape2 = *(iter++);
            b2Shape* oldShape3 = *(iter++);

            shapes.erase(oldShape1);
            testModel.removeShapes({oldShape1});

            REQUIRE(testModel.shapes().size() == shapes.size());
            for(b2Shape* s : testModel.shapes()) REQUIRE(shapes.count(s));

            shapes.erase(oldShape2);
            shapes.erase(oldShape3);
            testModel.removeShapes({oldShape2, oldShape3});

            REQUIRE(testModel.shapes().size() == shapes.size());
            for(b2Shape* s : testModel.shapes()) REQUIRE(shapes.count(s));

            delete oldShape1;
            delete oldShape2;
            delete oldShape3;
        }
    }
}
