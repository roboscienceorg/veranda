#include "diffDrive_drivetrain_plugin.h"
#include "diffDrive_drivetrain.h"

DiffDrive_Drivetrain_Plugin::DiffDrive_Drivetrain_Plugin()
{

}

DriveTrain_If* DiffDrive_Drivetrain_Plugin::createDrivetrain()
{
    return new DiffDrive_Drivetrain;
}
