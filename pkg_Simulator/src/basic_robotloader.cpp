#include "basic_robotloader.h"

#include <QDebug>
#include <Box2D/Box2D.h>
BasicRobotLoader::BasicRobotLoader(const QMap<QString, DriveTrain_Plugin_If*>& drivePlugs, const QMap<QString, Sensor_Plugin_If*>& sensorPlugs)
    : _drivetrains(drivePlugs), _sensors(sensorPlugs)
{

}

QString BasicRobotLoader::loadRobotFile(QString file, Robot *&output)
{
    if(_drivetrains.size())
    {
        b2CircleShape* circle = new b2CircleShape;
        circle->m_p.Set(0, 0);
        circle->m_radius = 10;

        b2EdgeShape* line = new b2EdgeShape;
        line->Set(b2Vec2(0, 0), b2Vec2(0, 10));

        //qDebug() << "Construct drivetrain";
        DriveTrain_If* dt;
        if(_drivetrains.contains("org.sdsmt.2dSim.drivetrain.floating"))
            dt = _drivetrains["org.sdsmt.2dSim.drivetrain.floating"]->createDrivetrain();
        else
            dt = _drivetrains.first()->createDrivetrain();

        //qDebug() << "Construct robot";
        output = new Robot({circle, line}, dt);
        return "";
    }
    return "No drivetrains defined";
}
