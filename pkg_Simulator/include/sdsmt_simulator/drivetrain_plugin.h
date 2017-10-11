#ifndef DRIVETRAIN_PLUGIN_H
#define DRIVETRAIN_PLUGIN_H

#include "drivetrain_if.h"

class DriveTrain_Plugin_If
{
public:
    virtual ~DriveTrain_Plugin_If(){}

    virtual DriveTrain_If* createDrivetrain() = 0;
};

Q_DECLARE_INTERFACE(DriveTrain_Plugin_If, "org.sdsmt.2dSim.drivetrain")

#endif // DRIVETRAIN_PLUGIN_H
