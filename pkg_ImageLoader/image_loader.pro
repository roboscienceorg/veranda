QT       += core

TARGET = diffDrive_drivetrain
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/image_loader.cpp \
    src/image_loader_plugin.cpp

HEADERS += \
    include/image_loader.h \
    include/image_loader_plugin.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
