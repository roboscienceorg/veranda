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
#include <QPluginLoader>
#include <QDir>
#include <QDirIterator>
#include <QDebug>
#include <QTextStream>
#include <QMap>

#include "basic_maploader.h"
#include "basic_physics.h"
#include "basic_robotloader.h"
#include "basic_ui.h"
#include "basic_viewer.h"
#include "ui/emptysimwindow.h"
#include "sdsmt_simulator/drivetrain_plugin.h"
#include "sdsmt_simulator/sensor_plugin.h"

#include "simulator_core.h"

using namespace std;

int main(int argc, char** argv)
{
    QTextStream cout(stdout);

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
    * Load robot part plugins
    *************************************/
    QMap<QString, DriveTrain_Plugin_If*> driveTrainPlugins;
    QMap<QString, Sensor_Plugin_If*> sensorPlugins;

    QPluginLoader plugLoader;

    //Minor hack which relies on the catkin output file structure to find plugins
    //checks all .so or .dll files in directory above executable
    //As of October, 2017, this is what ${CATKIN_PACKAGE_LIB_DESTINATION} points to
    //All packages for a workspace are put together in the same folders, so as long as all
    //plugins are set up as packages in the same workspace as this project, they should be found
    qInfo() << "Searching" << QCoreApplication::applicationDirPath() + "/.." << "for plugins";
    QDirIterator dir(QCoreApplication::applicationDirPath() + "/..", {"*.so", "*.dll"}, QDir::Files, QDirIterator::Subdirectories);
    while(dir.hasNext())
    {
        dir.next();
        plugLoader.setFileName(dir.fileInfo().absoluteFilePath());

        qInfo() << "Checking" << dir.fileInfo().absoluteFilePath() << "for plugin";

        if(plugLoader.load())
        {
            QObject* plugin = plugLoader.instance();
            QString iid = plugLoader.metaData()["IID"].toString();

            if(qobject_cast<DriveTrain_Plugin_If*>(plugin))
            {
                qInfo() << "Plugin type: Drivetrain";
                driveTrainPlugins[iid] = qobject_cast<DriveTrain_Plugin_If*>(plugin);
            }
            else if(qobject_cast<Sensor_Plugin_If*>(plugin))
            {
                qInfo() << "Plugin type: Sensor";
                sensorPlugins[iid] = qobject_cast<Sensor_Plugin_If*>(plugin);
            }
            else
            {
                qInfo() << "Unknown plugin type";
            }
        }
        else
        {
            qInfo() << plugLoader.errorString();
        }
    }
   /*************************************
    * Setup simulator
    *************************************/
    emptysimwindow::visualizerFactory visuals =
    []()
    {
        return new BasicViewer();
    };

    MapLoader_If* mapLoader = new BasicMapLoader();
    RobotLoader_If* robotLoader = new BasicRobotLoader(driveTrainPlugins, sensorPlugins);

    Simulator_Physics_If* physics = new BasicPhysics();
    Simulator_Ui_If* userinterface = new emptysimwindow(visuals);

    SimulatorCore sim(mapLoader, robotLoader, physics, userinterface, &app);

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
