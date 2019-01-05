#include "omni_drive_plugin.h"
#include "omni_drive.h"

WorldObjectComponent* Omni_Drive_Plugin::createComponent()
{
    return new Omni_Drive;
}
