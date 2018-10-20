QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include /opt/ros/ardent/include

SOURCES += \
    src/omni_drive.cpp \
    src/omni_drive_plugin.cpp

HEADERS += \
    include/omni_drive.h \
    include/omni_drive_plugin.h\
    include/defines.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
