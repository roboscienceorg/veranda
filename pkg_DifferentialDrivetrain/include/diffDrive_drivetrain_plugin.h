#ifndef FLOATER_DRIVETRAIN_PLUGIN_H
#define FLOATER_DRIVETRAIN_PLUGIN_H

#include <QObject>

#include <sdsmt_simulator/drivetrain_plugin.h>

class DiffDrive_Drivetrain_Plugin : public QObject, public DriveTrain_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.sdsmt.2dSim.drivetrain.differential")
    Q_INTERFACES(DriveTrain_Plugin_If)

public:
    DiffDrive_Drivetrain_Plugin();
    DriveTrain_If* createDrivetrain();

};

#endif // FLOATER_DRIVETRAIN_PLUGIN_H
