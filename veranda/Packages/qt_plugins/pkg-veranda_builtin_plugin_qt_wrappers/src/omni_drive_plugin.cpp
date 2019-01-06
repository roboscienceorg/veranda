#include "omni_drive_plugin.h"
#include "veranda_wheels/omni_drive.h"

WorldObjectComponent* Omni_Drive_Plugin::createComponent()
{
    return new Omni_Drive(OMNI_IID);
}
