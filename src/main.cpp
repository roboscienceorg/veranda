#include "ros/ros.h"

#include <string>
#include <functional>
#include <iostream>

#include <QTimer>
#include <QObject>
#include <QString>
#include <QApplication>
#include <QtConcurrent/QtConcurrent>
#include <QFuture>
#include <QFutureWatcher>

#include "basic_maploader.h"
#include "basic_physics.h"
#include "basic_robotloader.h"
#include "basic_ui.h"
#include "basic_viewer.h"
#include "ui/emptysimwindow.h"

#include "sdsmt_simulator.h"

using namespace std;

int main(int argc, char** argv)
{
   /*************************************
    * Setup ROS and Qt to play nice
    *************************************/
    //Init ros stuff
    ros::init(argc, argv, "sdsmt_simulator");

    //Init Qt
    QApplication app(argc, argv);

    //Start ros in separate thread, and trigger Qt shutdown when it exits
    //If Qt exits before ros, be sure to shutdown ros
    QFutureWatcher<void> rosThread;
    rosThread.setFuture(QtConcurrent::run(&ros::spin));
    QObject::connect(&rosThread, &QFutureWatcher<void>::finished, &app, &QCoreApplication::quit);
    QObject::connect(&app, &QCoreApplication::aboutToQuit, [](){ros::shutdown();});

   /*************************************
    * Setup simulator
    *************************************/
    emptysimwindow::visualizerFactory visuals =
    []()
    {
        return new BasicViewer();
    };

    MapLoader_If* mapLoader = new BasicMapLoader();
    RobotLoader_If* robotLoader = new BasicRobotLoader();

    Simulator_Physics_If* physics = new BasicPhysics();
    Simulator_Ui_If* userinterface = new emptysimwindow(visuals);

    SDSMT_Simulator sim(mapLoader, robotLoader, physics, userinterface, &app);

    sim.start();

   /******************
    * Run application
    ******************/
    //Start main app
    int ret = app.exec();

   /*******************
    * Clean up
    *******************/
    delete mapLoader;
    delete robotLoader;
    delete physics;
    delete userinterface;

    return ret;
}
