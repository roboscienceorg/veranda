QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/lidar_sensor.cpp \
    src/lidar_sensor_plugin.cpp

HEADERS += \
    include/lidar_sensor.h \
    include/lidar_sensor_plugin.h \
    include/defines.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
