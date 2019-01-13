QT       += core widgets

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include
include(../../../include_paths.pri)

SOURCES += \
    src/image_loader.cpp \
    src/image_loader_plugin.cpp \
    src/imageparser.cpp

HEADERS += \
    include/image_loader.h \
    include/image_loader_plugin.h \
    include/imageparser.h \
    include/optiondialog.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
