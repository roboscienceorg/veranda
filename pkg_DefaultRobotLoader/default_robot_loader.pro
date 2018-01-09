QT       += core

TARGET = diffDrive_drivetrain
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_Simulator/include

SOURCES += \
    src/default_robot_loader.cpp \
    src/default_robot_loader_plugin.cpp

HEADERS += \
    include/default_robot_loader.h \
    include/default_robot_loader_plugin.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
