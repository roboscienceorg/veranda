#include "Catch/catch.hpp"
#include "Box2D/Box2D.h"
#include "../include/veranda/model.h"

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

    SECTION("Transform defaults 0")
    {
        double x, y, t;
        testModel.getTransform(x, y, t);

        REQUIRE(x == Approx(0));
        REQUIRE(y == Approx(0));
        REQUIRE(t == Approx(0));
    }

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

            REQUIRE(count == i*2 + 1);
            REQUIRE(dx == Approx(x - xd));
            REQUIRE(dy == Approx(y - yd));
            REQUIRE(dt == Approx(t - td));

            xd = yd = td = 0;

            testModel.getTransform(xd, yd, td);
            testModel.setTransform(x, y, t);

            REQUIRE(count == i*2 + 2);
            REQUIRE(dx == Approx(0));
            REQUIRE(dy == Approx(0));
            REQUIRE(dt == Approx(0));
        }
    }
}

TEST_CASE("Get/Set Model Shapes")
{
    Model testModel;

    SECTION("Shapes default empty")
    {
        REQUIRE(testModel.shapes().size() == 0);
    }

    SECTION("Add shapes modifies result of Model::shapes()")
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

        qDeleteAll(testModel.shapes());
    }

    SECTION("Adding shapes twice does not create duplicates")
    {
        b2Shape* newShape = new b2EdgeShape();
        testModel.addShapes({newShape});
        testModel.addShapes({newShape});
        REQUIRE(testModel.shapes().size() == 1);
        delete newShape;
    }

    SECTION("Remove shapes modifies result of Model::shapes()")
    {
        std::unordered_set<b2Shape*> shapes;

        for(int i=0; i<3000; i++)
        {
            b2Shape* newShape1 = new b2EdgeShape();

            shapes.insert(newShape1);
            testModel.addShapes({newShape1});
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

    SECTION("Model::modelChanged()")
    {
        int modelChanges = 0;

        QObject::connect(&testModel, &Model::modelChanged,
        [&](Model* m){REQUIRE(m == &testModel); modelChanges++;});

        SECTION("Emitted when shapes added")
        {
            modelChanges = 0;

            for(int i=0; i<1000; i++)
            {
                b2Shape* newShape1 = new b2EdgeShape();
                b2Shape* newShape2 = new b2EdgeShape();
                b2Shape* newShape3 = new b2EdgeShape();

                testModel.addShapes({newShape1});

                REQUIRE(modelChanges == i*2 + 1);

                testModel.addShapes({newShape2, newShape3});

                REQUIRE(modelChanges == i*2 + 2);
            }

            qDeleteAll(testModel.shapes());
        }

        SECTION("Emitted when shape re-added")
        {
            b2Shape* newShape = new b2EdgeShape();

            testModel.addShapes({newShape});
            modelChanges = 0;
            testModel.addShapes({newShape});
            delete newShape;

            REQUIRE(modelChanges == 1);
        }

        SECTION("Emitted when shapes removed")
        {
            for(int i=0; i<3000; i++)
            {
                b2Shape* newShape1 = new b2EdgeShape();

                testModel.addShapes({newShape1});
            }

            modelChanges = 0;

            for(int i=0; i<1000; i++)
            {
                b2Shape* oldShape1 = testModel.shapes()[0];
                b2Shape* oldShape2 = testModel.shapes()[1];
                b2Shape* oldShape3 = testModel.shapes()[2];

                testModel.removeShapes({oldShape1});

                REQUIRE(modelChanges == i*2 + 1);

                testModel.removeShapes({oldShape2, oldShape3});

                REQUIRE(modelChanges == i*2 + 2);

                delete oldShape1;
                delete oldShape2;
                delete oldShape3;
            }
        }

        SECTION("Emitted when shape re-removed")
        {
            b2Shape* newShape = new b2EdgeShape();

            testModel.addShapes({newShape});
            testModel.removeShapes({newShape});
            modelChanges = 0;
            testModel.removeShapes({newShape});
            delete newShape;

            REQUIRE(modelChanges == 1);
        }
    }
}


TEST_CASE("Get/Set Model Children")
{
    Model testModel;

    SECTION("Children default empty")
    {
        REQUIRE(testModel.children().size() == 0);
    }

    SECTION("Add children modifies result of Model::children()")
    {
        std::unordered_set<Model*> models;

        for(int i=0; i<1000; i++)
        {
            Model* newModel1 = new Model();
            Model* newModel2 = new Model();
            Model* newModel3 = new Model();

            models.insert(newModel1);
            testModel.addChildren({newModel1});

            REQUIRE(testModel.children().size() == models.size());
            for(Model* s : testModel.children()) REQUIRE(models.count(s));

            models.insert(newModel2);
            models.insert(newModel3);
            testModel.addChildren({newModel2, newModel3});

            REQUIRE(testModel.children().size() == models.size());
            for(Model* s : testModel.children()) REQUIRE(models.count(s));
        }

        qDeleteAll(testModel.children());
    }

    SECTION("Adding models twice does not create duplicates")
    {
        Model* newModel = new Model();
        testModel.addChildren({newModel});
        testModel.addChildren({newModel});
        REQUIRE(testModel.children().size() == 1);
        delete newModel;
    }

    SECTION("Remove models modifies result of Model::models()")
    {
        std::unordered_set<Model*> models;

        for(int i=0; i<3000; i++)
        {
            Model* newModel1 = new Model();

            models.insert(newModel1);
            testModel.addChildren({newModel1});
        }

        for(int i=0; i<1000; i++)
        {
            auto iter = models.begin();
            Model* oldModel1 = *(iter++);
            Model* oldModel2 = *(iter++);
            Model* oldModel3 = *(iter++);

            models.erase(oldModel1);
            testModel.removeChildren({oldModel1});

            REQUIRE(testModel.children().size() == models.size());
            for(Model* s : testModel.children()) REQUIRE(models.count(s));

            models.erase(oldModel2);
            models.erase(oldModel3);
            testModel.removeChildren({oldModel2, oldModel3});

            REQUIRE(testModel.children().size() == models.size());
            for(Model* s : testModel.children()) REQUIRE(models.count(s));

            delete oldModel1;
            delete oldModel2;
            delete oldModel3;
        }
    }

    SECTION("Model::modelChanged()")
    {
        int modelChanges = 0;

        QObject::connect(&testModel, &Model::modelChanged,
        [&](Model* m){REQUIRE(m == &testModel); modelChanges++;});

        SECTION("Emitted when models added")
        {
            modelChanges = 0;

            for(int i=0; i<1000; i++)
            {
                Model* newModel1 = new Model();
                Model* newModel2 = new Model();
                Model* newModel3 = new Model();

                testModel.addChildren({newModel1});

                REQUIRE(modelChanges == i*2 + 1);

                testModel.addChildren({newModel2, newModel3});

                REQUIRE(modelChanges == i*2 + 2);
            }

            qDeleteAll(testModel.children());
        }

        SECTION("Emitted when Model re-added")
        {
            Model* newModel = new Model();

            testModel.addChildren({newModel});
            modelChanges = 0;
            testModel.addChildren({newModel});
            delete newModel;

            REQUIRE(modelChanges == 1);
        }

        SECTION("Emitted when models removed")
        {
            for(int i=0; i<3000; i++)
            {
                Model* newModel1 = new Model();

                testModel.addChildren({newModel1});
            }

            modelChanges = 0;

            for(int i=0; i<1000; i++)
            {
                Model* oldModel1 = testModel.children()[0];
                Model* oldModel2 = testModel.children()[1];
                Model* oldModel3 = testModel.children()[2];

                testModel.removeChildren({oldModel1});

                REQUIRE(modelChanges == i*2 + 1);

                testModel.removeChildren({oldModel2, oldModel3});

                REQUIRE(modelChanges == i*2 + 2);

                delete oldModel1;
                delete oldModel2;
                delete oldModel3;
            }
        }

        SECTION("Emitted when Model re-removed")
        {
            Model* newModel = new Model();

            testModel.addChildren({newModel});
            testModel.removeChildren({newModel});
            modelChanges = 0;
            testModel.removeChildren({newModel});
            delete newModel;

            REQUIRE(modelChanges == 1);
        }
    }
}

TEST_CASE("Force draw results in Model::modelChanged() signal")
{
    Model testModel;

    int modelChanges = 0;
    QObject::connect(&testModel, &Model::modelChanged, [&](Model* m){
        REQUIRE(m == &testModel); modelChanges++;
    });

    for(int i=0; i<10; i++)
        testModel.forceDraw();

    REQUIRE(modelChanges == 10);
}
