#include "image_loader_plugin.h"
#include "image_loader.h"

QVector<WorldLoader_If*> Image_Loader_Plugin::getLoaders()
{
    return QVector<WorldLoader_If*>{ new ImageLoader };
}
