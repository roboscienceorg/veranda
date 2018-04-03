#pragma once

#include <QObject>
#include <QDebug>

#include "Box2D/Box2D.h"

class ExampleClass1 : public QObject
{
  Q_OBJECT

public:
    ExampleClass1(QObject* parent=nullptr) : QObject(parent){}

signals:
    void iDidAThing(int*);

public slots:
    void youDidAThing(int* i)
    {
        (*i)++;
    }
};

class ExampleClass2 : public QObject
{
  Q_OBJECT

public:
    ExampleClass2(QObject* parent=nullptr) : QObject(parent){}

signals:
    void iDidAThing(int*);

public slots:
    void youDidAThing(int* i)
    {
        (*i)++;
    }
};