#ifndef SIMULATOR_UI_H
#define SIMULATOR_UI_H

#include <functional>

#include <QObject>
#include <QVector>
#include <QOpenGLWidget>

#include <Box2D/Box2D.h>

#include "robot.h"

class Simulator_Ui_If : public QObject
{
    std::function<QOpenGLWidget*()> _oglWidgetMaker;

protected:
    QOpenGLWidget* createOGLWidget(){return _oglWidgetMaker();}

public:
    Simulator_Ui_If(std::function<QOpenGLWidget*()> oglWidgetFactory, QObject* parent = nullptr) :
        QObject(parent), _oglWidgetMaker(oglWidgetFactory){}

signals:
    void simulatorWinOpened();
    void simulatorWinClosed();

    void mapWinOpened();
    void mapWinClosed();

    void robotWinOpened();
    void robotWinClosed();

    void physicsTickSet(double rate_hz, double duration_s);
    void physicsStopped();
    void physicsStarted();

    void robotLoadedIntoSimulation(QString robotFilePath);
    void robotUnloadedFromSimulation(QString robotFilePath);

    //Signal that the user switched the map
    void mapSetInSimulation(QString mapFilePath);

public slots:
    void robotAddedToSimulation(Robot_Properties* robot, robot_id rId);
    void robotRemovedFromSimulation(robot_id rId);
}

#endif // SIMULATOR_UI_H
