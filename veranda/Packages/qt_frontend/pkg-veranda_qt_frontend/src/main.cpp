#include "rclcpp/rclcpp.hpp"

#include <string>
#include <functional>
#include <iostream>
#include <memory>

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
#include <QFileDialog>

#include "ui/qgraphicssimulationviewer.h"

#include <veranda_core/basic_physics.h>
#include <veranda_core/simulator_core.h>
#include <veranda_core/interfaces/world_object_component_factory_if.h>

#include <veranda_qt_frontend/file_handler_plugin.h>
#include <veranda_qt_plugins/world_object_component_plugin.h>

//This MUST be the last include
//For some reason, MSVC will hate you forever if
//QOpenGLWidget is included before some of the other
//stuff
#include "ui/mainwindow.h"

using namespace std;

int main(int argc, char** argv)
{
    QTextStream cout(stdout);

   /*************************************
    * Setup ROS and Qt to play nice
    *************************************/
    //Init ros stuff
    rclcpp::init(argc, argv);
    shared_ptr<rclcpp::Node> node = make_shared<rclcpp::Node>("veranda");

    //Init Qt
    QApplication app(argc, argv);

    //Start ros in separate thread, and trigger Qt shutdown when it exits
    //If Qt exits before ros, be sure to shutdown ros

    //Removed for now because multi-threading isn't stable in ROS2 yet
    //Replaced by timer below

    
    /*-----------------ORIGINAL MULTITHREADING------------------
    QFutureWatcher<void> rosThread;
    rosThread.setFuture(QtConcurrent::run([node](){rclcpp::spin(node);}));
    QObject::connect(&rosThread, &QFutureWatcher<void>::finished, &app, &QCoreApplication::quit);
    QObject::connect(&app, &QCoreApplication::aboutToQuit, [](){rclcpp::shutdown();});
    */
    //-----------------TIMER REPLACEMENT HACK--------------------
    //TODO: Remove when ROS2 is threadsafe
    //and it is safe to spin() in one thread and create/destroy publishers/subscribers
    //from another
    QTimer spinTimer;
    spinTimer.setInterval(30);
    spinTimer.setTimerType(Qt::PreciseTimer);
    QObject::connect(&spinTimer, &QTimer::timeout,
    [&]()
    {
        if(!rclcpp::ok())
        {
            spinTimer.stop();
            app.quit();
        }
        else
        {
            rclcpp::spin_some(node);
        }
    });
    QObject::connect(&app, &QCoreApplication::aboutToQuit, [](){rclcpp::shutdown();});
    spinTimer.start();
    //-----------------------------------------------------------

   /*************************************
    * Load robot part plugins
    *************************************/
    QMap<QString, WorldObjectComponent_Factory_If*> componentPlugins;
    QVector<WorldObjectLoader_If*> objectLoaders;
    QVector<WorldObjectSaver_If*> objectSavers;
    QVector<WorldLoader_If*> worldLoaders;
    QVector<WorldSaver_If*> worldSavers;
    WorldLoader_If* defaultLoader = nullptr;

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

            if(iid == "org.roboscience.veranda.fileHandlers.defaultBot")
            {
                qInfo() << "Found default robot loader";
                defaultLoader = qobject_cast<WorldFileHandler_Plugin_If*>(plugin)->getLoaders()[0];
            }
            else if(qobject_cast<WorldObjectComponent_Plugin_If*>(plugin))
            {
                qInfo() << "Component Plugin Accepted";
                componentPlugins[iid] = qobject_cast<WorldObjectComponent_Plugin_If*>(plugin);
            }
            else if(qobject_cast<WorldObjectFileHandler_Plugin_If*>(plugin))
            {
                qInfo() << "Object Filehandler Plugin Accepted";
                WorldObjectFileHandler_Plugin_If* p = qobject_cast<WorldObjectFileHandler_Plugin_If*>(plugin);
                objectLoaders += p->getLoaders();
                objectSavers += p->getSavers();
            }
            else if(qobject_cast<WorldFileHandler_Plugin_If*>(plugin))
            {
                qInfo() << "World Filehandler Plugin Accepted";
                WorldFileHandler_Plugin_If* p = qobject_cast<WorldFileHandler_Plugin_If*>(plugin);
                worldLoaders += p->getLoaders();
                worldSavers += p->getSavers();
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
    MainWindow::visualizerFactory visuals =
    []()
    {
        return new QGraphicsSimulationViewer();
    };

    //Get rid of defaultLoader so
    //default robots aren't used
    defaultLoader = nullptr;

    Simulator_Physics_If* physics = new BasicPhysics;
    Simulator_Ui_If* userinterface = new MainWindow(visuals, componentPlugins, objectLoaders, objectSavers, worldLoaders, worldSavers, defaultLoader);

    SimulatorCore sim(physics, userinterface, node, &app);

    if(defaultLoader)
    {
        auto def = defaultLoader->loadFile("", componentPlugins);
        sim.addSimObjects(def, false);
    }

    qDebug() << "Starting Simulation";
    sim.start();

   /******************
    * Run application
    ******************/
    //Start main app
    int ret = app.exec();

    return ret;
}
