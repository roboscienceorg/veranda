#include "image_loader_plugin.h"
#include "image_loader.h"

Image_Loader_Plugin::Image_Loader_Plugin()
{

}

QVector<WorldObjectLoader_If*> Image_Loader_Plugin::getLoaders()
{
    return QVector<WorldObjectLoader_If*>{ new ImageLoader };
}
