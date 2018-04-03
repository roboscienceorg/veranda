#include "exampleheader.h"
#include "Catch/catch.hpp"

#include <QDebug>

TEST_CASE("Examples")
{
    int j=0;
    ExampleClass1* c1 = new ExampleClass1();
    ExampleClass2* c2 = new ExampleClass2();

    b2PolygonShape poly;

    qDebug() << "Connect 1" << QObject::connect(c1, &ExampleClass1::iDidAThing, c2, &ExampleClass2::youDidAThing);
    qDebug() << "Connect 2" << QObject::connect(c2, &ExampleClass2::iDidAThing, c1, &ExampleClass1::youDidAThing);

    SECTION("Can signal from thing 1")
    {
        for(int i=0; i<10; i++)
            c1->iDidAThing(&j);
        REQUIRE(j==10);
    }

    SECTION("Can signal from thing 2")
    {
        for(int i=0; i<10; i++)
            c2->iDidAThing(&j);
        REQUIRE(j==10);
    }
}
