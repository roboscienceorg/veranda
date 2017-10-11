#include "floating_drivetrain_plugin.h"
#include "floating_drivetrain.h"

Floating_DriveTrain_Plugin::Floating_DriveTrain_Plugin()
{

}

DriveTrain_If* Floating_DriveTrain_Plugin::createDrivetrain()
{
    return new Floating_Drivetrain;
}
