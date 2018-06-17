QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/default_robot_loader.cpp \
    src/default_robot_loader_plugin.cpp

HEADERS += \
    include/default_robot_loader.h \
    include/default_robot_loader_plugin.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
