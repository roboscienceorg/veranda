#include "floating_drivetrain_plugin.h"
#include "floating_drivetrain.h"

Floating_Drivetrain_Plugin::Floating_Drivetrain_Plugin()
{

}

DriveTrain_If* Floating_Drivetrain_Plugin::createDrivetrain()
{
    return new Floating_Drivetrain;
}
