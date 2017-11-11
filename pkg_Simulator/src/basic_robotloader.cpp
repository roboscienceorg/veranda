#include "basic_robotloader.h"

#include <QDebug>
#include <Box2D/Box2D.h>
#include <cmath>
#include <ctime>

BasicRobotLoader::BasicRobotLoader(const QMap<QString, DriveTrain_Plugin_If*>& drivePlugs, const QMap<QString, Sensor_Plugin_If*>& sensorPlugs)
    : _drivetrains(drivePlugs), _sensors(sensorPlugs)
{
    srand(time(nullptr));
}

QString BasicRobotLoader::loadRobotFile(QString file, Robot *&output)
{
    if(_drivetrains.size())
    {
        b2CircleShape* circle = new b2CircleShape;
        circle->m_p.Set(0, 0);
        circle->m_radius = 0.5;

        b2EdgeShape* line = new b2EdgeShape;
        line->Set(b2Vec2(0, 0), b2Vec2(0.5, 0));

        //qDebug() << "Construct drivetrain";
        DriveTrain_If* dt;
        if(_drivetrains.contains("org.sdsmt.2dSim.drivetrain.differential"))
            dt = _drivetrains["org.sdsmt.2dSim.drivetrain.differential"]->createDrivetrain();
        else
            dt = _drivetrains.first()->createDrivetrain();

        //qDebug() << "Construct robot";
        output = new Robot({circle, line}, dt, rand() % 30, rand() % 30, rand()%360);
        return "";
    }
    return "No drivetrains defined";
}
