#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <functional>
#include <iostream>

#include <QCoreApplication>
#include <QTimer>
#include <QObject>
#include <QGuiApplication>
#include <QUrl>
#include <QString>
#include <QApplication>
#include <QtConcurrent/QtConcurrent>
#include <QFuture>
#include <QFutureWatcher>

using namespace std;

int main(int argc, char** argv)
{
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

    //5 second timer to publish
    QTimer sec5;
    sec5.setInterval(5000);

    //Set up slot for 5 second timer
    int i=0;
    QObject::connect(&sec5, &QTimer::timeout, [&]()
    {
        cout << ros::ok() << endl;
    });

    //Start timer
    sec5.start();

    //Start main app
    return app.exec();
}
