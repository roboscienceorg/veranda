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

#include "basic_physics.h"
#include "basic_viewer.h"
#include "ui/mainwindow.h"

#include <sdsmt_simulator/world_object_component_plugin.h>
#include <sdsmt_simulator/world_object_file_handler_plugin.h>

#include "simulator_core.h"

using namespace std;

int main(int argc, char** argv)
{
    QTextStream cout(stdout);

   /*************************************
    * Setup ROS and Qt to play nice
    *************************************/
    //Init ros stuff
    rclcpp::init(argc, argv);
    shared_ptr<rclcpp::Node> node = make_shared<rclcpp::Node>("sdsmt_simulator");

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
    spinTimer.setInterval(0.01);
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
    QMap<QString, WorldObjectComponent_Plugin_If*> componentPlugins;
    QVector<WorldObjectLoader_If*> objectLoaders;
    QVector<WorldObjectSaver_If*> objectSavers;
    WorldObjectLoader_If* imageLoader = nullptr;

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

            if(qobject_cast<WorldObjectComponent_Plugin_If*>(plugin))
            {
                qInfo() << "Component Plugin Accepted";
                componentPlugins[iid] = qobject_cast<WorldObjectComponent_Plugin_If*>(plugin);
            }
            else if(qobject_cast<WorldObjectFileHandler_Plugin_If*>(plugin))
            {
                qInfo() << "Filehandler Plugin Accepted";
                WorldObjectFileHandler_Plugin_If* p = qobject_cast<WorldObjectFileHandler_Plugin_If*>(plugin);
                objectLoaders += p->getLoaders();
                objectSavers += p->getSavers();

                if(iid == "org.sdsmt.sim.2d.fileHandlers.imageLoader")
                    imageLoader = p->getLoaders()[0];
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
        return new BasicViewer;
    };

    Simulator_Physics_If* physics = new BasicPhysics;
    Simulator_Ui_If* userinterface = new MainWindow(visuals, componentPlugins, objectLoaders, objectSavers);

    SimulatorCore sim(physics, userinterface, node, &app);

    if(imageLoader)
    {
        QString fileName = QFileDialog::getOpenFileName();
        for(WorldObject* wo : imageLoader->loadFile(fileName, componentPlugins))
        {
            sim.addSimObject(wo);
            delete wo;
        }
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
