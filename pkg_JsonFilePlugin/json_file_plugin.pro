QT       += core

TARGET = diffDrive_drivetrain
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/json_object.cpp \
    src/json_object_plugin.cpp \
    src/json_world.cpp \
    src/json_world_plugin.cpp

HEADERS += \
    include/json_object.h \
    include/json_object_plugin.h \
    include/json_world.h \
    include/json_world_plugin.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
