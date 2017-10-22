#ifndef BASIC_ROBOTLOADER_H
#define BASIC_ROBOTLOADER_H

#include "sdsmt_simulator/drivetrain_plugin.h"
#include "sdsmt_simulator/sensor_plugin.h"

#include "interfaces/robot_loader_if.h"

#include <QVector>
#include <QMap>
#include <QString>

class BasicRobotLoader : public RobotLoader_If
{
    QMap<QString, DriveTrain_Plugin_If*> _drivetrains;
    QMap<QString, Sensor_Plugin_If*> _sensors;

public:
    BasicRobotLoader(const QMap<QString, DriveTrain_Plugin_If*>& drivePlugs, const QMap<QString, Sensor_Plugin_If*>& sensorPlugs);
    ~BasicRobotLoader(){}

    virtual QString loadRobotFile(QString file, Robot*& output) override;
};
#endif // BASIC_ROBOTLOADER_H
